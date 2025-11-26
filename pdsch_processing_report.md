# PDSCH Processing Chain Analysis

This document provides a detailed analysis of the PDSCH (Physical Downlink Shared Channel) processing chain within the cuPHY-CP architecture, tracing the flow from L2-MAC FAPI messages to O-RAN Fronthaul U-plane packets.

## 1. High-Level Overview

The PDSCH pipeline transforms high-level scheduling requests from the MAC layer into physical signals transmitted over the air. The process involves:
1.  **FAPI Ingress**: Receiving and parsing `DL_TTI_REQUEST` and `TX_DATA_REQUEST` messages.
2.  **Task Dispatch**: Translating FAPI messages into internal cuPHY tasks and enqueueing them for execution.
3.  **GPU Execution**: Offloading heavy signal processing (encoding, modulation, mapping) to the GPU via `PhyPdschAggr`.
4.  **Compression**: Compressing IQ samples for efficient fronthaul transmission.
5.  **Fronthaul Egress**: Packetizing compressed data into O-RAN U-plane messages and transmitting them via the NIC.

## 2. FAPI Ingress

The entry point for PDSCH processing is the handling of FAPI messages in `scf_5g_fapi_phy.cpp`.

### 2.1 DL_TTI_REQUEST
The `on_dl_tti_request` function processes the `DL_TTI_REQUEST` message, which contains scheduling information for a specific slot.
-   **Parsing**: Iterates through PDUs (Protocol Data Units). For `DL_TTI_PDU_TYPE_PDSCH`, it extracts PDSCH configuration (RNTI, modulation, resource allocation).
-   **Command Preparation**: Calls `prepare_dl_slot_command` -> `update_cell_command` to populate the `pdsch_params` structure within the `cell_group_command`.
-   **Key Data**: Stores pointers to PDSCH parameters but *not* the payload data yet.

### 2.2 TX_DATA_REQUEST
The `on_phy_dl_tx_request` function handles the `TX_DATA_REQUEST` message, which carries the actual Transport Block (TB) data.
-   **Data Association**: Matches the data PDUs with the previously received PDSCH configuration.
-   **Buffer Management**:
    -   If `prepone_h2d_copy` is enabled, it may initiate an early Host-to-Device (H2D) copy.
    -   Updates `pdsch_params->ue_tb_ptr` with the address of the data buffer.
    -   Sets `pdsch_params->tb_data.pBufferType` to indicate if data is in CPU memory or already on the GPU (`cuphyPdschDataIn_t::GPU_BUFFER`).

## 3. Task Dispatch

The `l1_enqueue_phy_work` function in `cuphydriver_api.cpp` orchestrates the execution flow. It translates the accumulated slot commands into a set of interdependent tasks.

### 3.1 Task Creation
For PDSCH, the following tasks are relevant:
-   **`task_work_function_dl_aggr_1_pdsch`**: The main PDSCH processing task.
-   **`task_work_function_dl_aggr_1_compression`**: Handles data compression.
-   **`task_work_function_dl_aggr_2_gpu_comm`** (or `_2`): Handles fronthaul packet preparation and transmission.

### 3.2 Dependency Management
Tasks are linked via a `SlotMapDl` object, which manages synchronization primitives (events, barriers).
-   **PDSCH Task** depends on the completion of H2D copies (if not preponed).
-   **Compression Task** depends on the completion of the PDSCH task (generation of IQ samples).
-   **Fronthaul Task** depends on the completion of the Compression task.

## 4. PDSCH Task Execution

The `PhyPdschAggr` class (`phypdsch_aggr.cpp`) manages the GPU execution.

### 4.1 Setup (`setup`)
-   **Parameter Aggregation**: Aggregates PDSCH parameters from multiple cells (if Carrier Aggregation is active) into a single batch.
-   **Resource Allocation**: Allocates GPU memory for intermediate buffers if needed.
-   **cuPHY Configuration**: Calls `cuphySetupPdschTx` to configure the cuPHY kernel parameters.

### 4.2 Execution (`run`)
-   **Kernel Launch**: Calls `cuphyRunPdschTx` to launch the CUDA kernels.
    -   **Input**: TB data, DMRS configuration, PRB allocation.
    -   **Output**: Uncompressed IQ samples in the `DLOutputBuffer`.
-   **Synchronization**: Records a CUDA event (`PdschDone`) to signal completion.

## 5. Compression

The `task_work_function_dl_aggr_1_compression` handles the compression of IQ samples.
-   **Input**: Uncompressed IQ samples from `DLOutputBuffer`.
-   **Operation**: Launches a compression kernel (e.g., Block Floating Point) to reduce data volume.
-   **Output**: Compressed IQ samples in the `DLOutputBuffer`'s transmission buffer.
-   **Synchronization**: Waits for `PdschDone` event before starting. Records `CompressionDone` event upon completion.

## 6. Fronthaul Egress

The transmission of U-plane packets is handled by `FhProxy` (`fh.cpp`), triggered by `task_work_function_dl_aggr_2_gpu_comm` (for GPU-centric mode).

### 6.1 Packet Preparation (`prepareUPlanePackets`)
-   **Header Generation**: Creates O-RAN C-plane and U-plane headers based on the slot configuration and PDSCH mapping.
-   **Section Management**: Divides the frequency domain resources into sections (e.g., based on beamforming weights or compression parameters).
-   **GPU Communication**: If `gpuCommDlEnabled` is true, it calls `prepare_uplane_gpu_comm` to set up the GPU-controlled packet generation. This involves preparing descriptors that the GPU will use to write headers and data directly to the NIC's TX ring.

### 6.2 Transmission (`UserPlaneSendPacketsGpuComm`)
-   **Trigger**: Calls `aerial_fh::send_uplane_gpu_comm`.
-   **Mechanism**:
    -   The GPU kernel (triggered by the driver) writes the packet headers and payload (compressed IQ samples) into the NIC's memory.
    -   It updates the NIC's doorbell to notify it of new packets.
-   **Synchronization**: Ensures that transmission only starts after compression is complete (`CompressionDone` event).

## 7. Parallelism and Synchronization

The architecture relies heavily on asynchronous execution and CUDA streams/events to maximize throughput.

-   **Pipeline Parallelism**: CPU tasks (FAPI parsing, header prep) run in parallel with GPU tasks (PDSCH processing).
-   **Data Parallelism**: Multiple cells are processed concurrently within the same `PhyPdschAggr` task (Carrier Aggregation).
-   **Stream Ordering**:
    -   Stream 1: H2D Copy -> PDSCH Kernel -> Compression Kernel.
    -   Stream 2: Fronthaul Header Generation (CPU) -> GPU Descriptor Update.
-   **Synchronization Points**:
    -   `wait_event(PdschDone)`: Compression waits for PDSCH.
    -   `wait_event(CompressionDone)`: Fronthaul transmission waits for compression.

## 8. Summary of Data Flow

1.  **L2-MAC**: Sends `DL_TTI_REQUEST` (Config) and `TX_DATA_REQUEST` (Payload).
2.  **CPU (SCF Adapter)**: Parses FAPI, updates `pdsch_params` with config and data pointer.
3.  **CPU (Driver)**: Enqueues `PDSCH`, `Compression`, and `FH` tasks.
4.  **GPU (PDSCH Task)**: Reads TB data, generates IQ samples -> `DLOutputBuffer`.
5.  **GPU (Compression Task)**: Reads IQ samples, compresses them -> `DLOutputBuffer` (TX Buffer).
6.  **CPU/GPU (FH Task)**: Prepares headers, triggers NIC transmission from `DLOutputBuffer`.
7.  **NIC**: Sends O-RAN U-plane packets to the Radio Unit (RU).
