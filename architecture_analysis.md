# cuBB Architecture Deep Dive Analysis

## 1. Executive Summary

The cuBB (CUDA Baseband) codebase implements a high-performance, GPU-accelerated 5G PHY layer compliant with O-RAN specifications. It leverages a hybrid CPU-GPU architecture where latency-critical control logic runs on pinned CPU cores, while heavy signal processing (LDPC, FFT, Channel Estimation) is offloaded to NVIDIA GPUs. The system is designed for determinism, utilizing "Green Contexts" (MPS) for GPU resource isolation and real-time scheduling policies (SCHED_FIFO) for CPU threads.

## 2. System Architecture Overview

### 2.1 High-Level Components

*   **cuPHY Controller (L2 Adapter)**: Acts as the interface between the MAC layer (L2) and the PHY layer (L1). It handles FAPI (Functional Application Programming Interface) messaging, slot synchronization, and configuration management.
*   **cuPHY Driver**: The core engine responsible for scheduling and executing PHY tasks. It manages worker threads, GPU resources, and the execution pipeline.
*   **cuPHY Library**: A collection of CUDA kernels optimized for 5G PHY signal processing (e.g., PUSCH, PDSCH, PRACH).
*   **Fronthaul (FH) Interface**: Handles O-RAN 7.2x split communication, managing U-plane data transfer via DPDK.

### 2.2 Hardware Abstraction

*   **CPU**: Utilizes specific cores for distinct roles (Worker, Polling, OAM). Thread affinity is strictly managed via configuration (YAML).
*   **GPU**: Treated as a pool of Streaming Multiprocessors (SMs). Work is submitted via CUDA streams, often overlapping copy and compute engines.
*   **NIC**: ConnectX-6/7 DX cards used for high-speed fronthaul packet I/O (GPUDirect RDMA is implied for data path).

## 3. Threading and Execution Model

The system employs a **Worker Pool** pattern with strict affinity and priority settings to minimize jitter.

### 3.1 Thread Roles

| Thread Name | Role | Scheduling | Implementation |
| :--- | :--- | :--- | :--- |
| `msg_processing` | **L2 Adapter Main Loop**. Receives FAPI messages, tracks slot time, and dispatches slot commands to the driver. | SCHED_FIFO (High) | `PHY_module::thread_func` |
| `WorkerUL` | **Uplink Worker**. Pulls UL tasks (PRACH, PUSCH) from a queue and submits them to the GPU. | SCHED_FIFO (High) | `Worker::run` |
| `WorkerDL` | **Downlink Worker**. Pulls DL tasks (PDSCH, PDCCH) and submits them to the GPU. | SCHED_FIFO (High) | `Worker::run` |
| `oam_cell_update` | **OAM Handler**. Processes non-real-time configuration updates. | SCHED_OTHER (Low) | `PHY_module::cell_update_thread_func` |
| `sfn_slot_sync` | **Sync Handler**. Manages SFN/Slot synchronization commands. | SCHED_OTHER (Low) | `PHY_module::sfn_slot_sync_cmd_thread_func` |

### 3.2 Worker Implementation Details (`worker.cpp`)

*   **Initialization**: Workers are initialized with a specific type (UL/DL), CPU core affinity, and priority.
*   **Main Loop**:
    1.  **Task Retrieval**: Calls `tList->get_task()` with a timeout (`acceptns`).
    2.  **Execution**: Calls `nTask->run(worker_instance)`.
    3.  **Metrics**: Collects PMU (Performance Monitor Unit) data and execution timestamps.
    4.  **Stall Simulation**: Can inject artificial CPU stalls for stress testing (`simulated_cpu_stall_checkpoint`).

## 4. Data Flow and Pipeline

### 4.1 Downlink (DL) Data Flow

1.  **L2 Ingress**: FAPI `DL_TTI.request` and `TX_DATA.request` messages arrive via IPC.
2.  **Buffering**: `PHY_module::recv_msg` receives messages. Data is buffered in `dl_tbs_queue_`.
3.  **Slot Trigger**: When all cells receive `SLOT.response` (or based on timer), `PHY_module::process_phy_commands` is triggered.
4.  **Dispatch**:
    *   `process_phy_commands` constructs a `slot_command`.
    *   Calls `PHYDriverProxy::l1_enqueue_phy_work(slot_cmd)`.
    *   Driver creates `TaskDL...` objects (e.g., `TaskDL1AggrPdsch`) and pushes them to the DL TaskList.
5.  **Execution**:
    *   `WorkerDL` wakes up, pops the task.
    *   `Task::run` invokes cuPHY kernels (e.g., Modulation, Layer Mapping, Precoding).
    *   Output is written to GPU buffers.
6.  **Fronthaul**: Compressed IQ samples are sent to the O-RU via the Fronthaul interface.

### 4.2 Uplink (UL) Data Flow

1.  **Fronthaul Ingress**: O-RU sends IQ samples. FH interface writes them to memory (often GPU memory via GPUDirect).
2.  **Slot Indication**: `PHY_module::tick_received` generates `SLOT.indication` to L2.
3.  **L2 Response**: L2 sends `UL_TTI.request`.
4.  **Dispatch**:
    *   `PHY_module` processes the request and enqueues UL tasks via `l1_enqueue_phy_work`.
    *   Tasks include `TaskUL1AggrPrach`, `TaskUL1AggrPucchPusch`.
5.  **Execution**:
    *   `WorkerUL` pops the task.
    *   `PhyPrachAggr::run` or `PhyPuschAggr::run` is called.
    *   **GPU Kernel**: `cuphyRunPrachRx` / `cuphyRunPuschRx` executes on the GPU.
6.  **Completion**:
    *   Results (CRC, UCI, RX Data) are copied back to CPU.
    *   Callback `PHY_module::on_ul_tb_processed` sends FAPI indications (CRC.ind, RX_DATA.ind) back to L2.

## 5. Synchronization and Latency Management

### 5.1 Latency Protection
*   **Threshold Check**: `PHY_module::check_time_threshold` verifies if the slot processing is starting too late (Current Time > Slot Time + 500us + Allowed Latency). If so, the slot is **dropped** to prevent cascading failures.
*   **Busy-Wait vs. Epoll**: The system supports both `epoll` (event-driven) and busy-wait loops for message processing, configurable via `ENABLE_L2_SLT_RSP` and build flags.

### 5.2 Locking Strategy
*   **`tick_lock`**: Protects the shared SFN/Slot counters (`ss_tick`, `current_tick_`) accessed by the tick generator and the main processing loop.
*   **`dl_tbs_lock`**: Protects the queue of Downlink Transport Blocks (`dl_tbs_queue_`) during enqueue (IPC thread) and dequeue (Driver thread).
*   **`TaskList` Lock**: A mutex protects the worker task queues, ensuring thread-safe task stealing/retrieval.

## 6. GPU Resource Management (MPS & Green Contexts)

The system explicitly manages GPU resources to ensure isolation between cells or tasks.

*   **Legacy MPS**: Uses `cuCtxCreate_v3` with `CU_EXEC_AFFINITY_TYPE_SM_COUNT` to limit the number of SMs a context can use.
*   **Green Contexts (Newer CUDA)**: Uses `cuGreenCtxCreate` (verified in `mps.cpp`) to create lightweight, isolated contexts with strict SM partitioning. This is critical for:
    *   **Predictability**: Preventing one cell's heavy load from affecting another's latency.
    *   **QoS**: Guaranteering resources for high-priority channels (e.g., URLLC).

## 7. Key Class Hierarchy

*   **`PHY_module`**: The "Brain". Manages lifecycle, IPC, and time.
    *   Has `phy_instances_` (vector of `PHY_instance_ptr`).
    *   Has `transport_wrapper_` for IPC.
*   **`PhyDriverCtx`**: The "Engine Room". Holds global driver state.
    *   Manages `GpuDevice` list.
    *   Holds `TaskList`s (UL, DL, Debug).
*   **`Worker`**: The "Muscle". Generic thread wrapper.
    *   Executes `Task` objects.
*   **`Task`**: Unit of work.
    *   Subclasses: `SlotMapUl`, `SlotMapDl` (contain slot-specific params).
    *   Wraps aggregators like `PhyPrachAggr`.
*   **`PhyPrachAggr` / `PhyPdschAggr`**: Channel-specific logic.
    *   Configures cuPHY descriptors.
    *   Launches specific CUDA kernels.


## 7. Task Management Architecture

The cuPHY-CP uses a custom task management system designed for low-latency, real-time execution.

### 7.1 Task Structure
-   **`Task` Class**: Represents a unit of work. Key attributes include:
    -   `ts_exec`: Execution timestamp (when the task *should* run).
    -   `work_f`: Function pointer to the task logic.
    -   `desired_wid`: Optional worker ID preference (affinity).
    -   `priority`: Scheduling priority.
-   **`TaskList`**: Manages a collection of tasks using `task_priority_queue`.
    -   Maintains separate priority queues for each worker ID and a generic queue for any worker.
    -   Tasks are ordered by `ts_exec`.

### 7.2 Scheduling & Execution
-   **Worker Threads**: `Worker` objects run in a loop (`worker_default`), pinned to specific CPU cores.
-   **Pull Model**: Workers actively poll the `TaskList` (`tList->get_task`).
    -   A worker first checks its specific queue, then the generic queue.
    -   Tasks are only dequeued if `Time::nowNs() >= task->ts_exec`.
-   **Real-Time Behavior**: If no task is ready, workers sleep briefly (`std::this_thread::sleep_for`) to yield CPU, but the polling interval is tight to minimize latency.

### 7.3 Synchronization (`SlotMap`)
-   **Context**: `SlotMapDl` (Downlink) and `SlotMapUl` (Uplink) serve as the shared context for all tasks within a slot.
-   **Mechanism**: Uses `std::atomic` counters and flags for lightweight synchronization.
    -   Example: `atom_dlc_done` tracks completed compression tasks.
    -   **Waiters**: Functions like `waitDLCDone()` spin/yield until dependencies are met, allowing tasks to coordinate without heavy kernel-level blocking.
-   **Lifecycle**: The `SlotMap` is created at the start of a slot and released when all tasks are complete.

## 8. Conclusion

The cuBB architecture is a sophisticated example of a real-time, GPU-accelerated system. Its design prioritizes low latency and determinism through strict thread pinning, lock-free data paths where possible (or minimal locking), and explicit GPU resource partitioning. The separation of "Controller" (L2 Adapter) and "Driver" allows for a clean decoupling of protocol logic from hardware execution.
