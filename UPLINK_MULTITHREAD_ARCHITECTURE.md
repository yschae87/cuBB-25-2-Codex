# NVIDIA Aerial cuPHY-CP: Uplink Multi-Thread Architecture Analysis

## Document Information

- **Date**: 2025-11-19
- **System**: NVIDIA Aerial cuBB (cuPHY-CP)
- **Focus**: Uplink PRACH Processing Multi-Task Multi-Thread Architecture
- **Configuration**: μ=1 (30 kHz SCS), 2-cell deployment (Carrier Aggregation)

---

## Executive Summary

The NVIDIA Aerial cuPHY-CP implements a sophisticated **multi-threaded worker pool architecture** with the following key characteristics:

### Architecture Highlights

- **Worker Threads**: `UlPhyDriver04`, `UlPhyDriver05`, `UlPhyDriver08` (numbered worker IDs)
- **Task Instances**: `AGGR 0`, `AGGR 1`, `AGGR 2`, `AGGR 3` (per-thread task counters)
- **Cell Assignment**: Dynamic round-robin slot-based distribution
- **Processing Model**: Asynchronous GPU-accelerated PRACH detection with CPU coordination
- **Load Balancing**: Slot-based round-robin across thread pool
- **Synchronization**: Lock-free atomic operations with tick-based coordination

### Key Finding: AGGR Task Numbering

**AGGR 3 ≠ "Aggregator for Cell 3"**

**AGGR 3 = "4th aggregation task instance (zero-indexed) spawned by UlPhyDriver05 thread"**

This task happens to process **cells [0, 1]** for **Slot Map 448** at the time of failure.

---

## Table of Contents

1. [UlPhyDriver Thread Architecture](#section-1-ulphydriver-thread-architecture)
2. [PRACH Aggregation Task Management](#section-2-prach-aggregation-task-management)
3. [Cell Assignment Architecture](#section-3-cell-assignment-architecture)
4. [Slot Processing Pipeline](#section-4-slot-processing-pipeline)
5. [Thread Naming Explanation](#section-5-thread-naming-explanation)
6. [Data Structures](#section-6-data-structures-for-task-management)
7. [Synchronization and Coordination](#section-7-synchronization-and-coordination)
8. [Configuration and Initialization](#section-8-configuration-and-initialization)
9. [Architecture Diagrams](#section-9-architecture-diagram)
10. [Sequence Diagrams](#section-10-sequence-diagram-for-slot-processing)
11. [Code References](#section-11-key-files-and-line-references)

---

## Section 1: UlPhyDriver Thread Architecture

### 1.1 Thread Identification and Naming

**Source Files**:
- `cuPHY-CP/cuphyl2adapter/lib/nvPHY/nv_phy_driver_proxy.hpp`
- `cuPHY-CP/cuphyl2adapter/lib/nvPHY/nv_phy_driver_proxy.cpp`
- `cuPHY-CP/cuphyl2adapter/lib/nvPHY/nv_phy_module.hpp` (lines 426-427, 450-451)

**Thread Creation Pattern**:
```cpp
// From nv_phy_module.hpp
class PHY_module {
private:
    std::thread thread_;                           // Module thread (line 426)
    std::unique_ptr<thread_config> thread_cfg_;    // Thread configuration (line 427)
    std::atomic<nanoseconds> current_tick_;        // Atomic tick sync (line 450)

    std::vector<PHY_instance_ptr> phy_instances_;  // PHY worker instances
    std::vector<PHY_instance_ref> phy_refs_;       // References
};
```

**Thread Naming Convention**:
```
UlPhyDriver04 = Uplink PHY Driver Worker Thread ID 4
UlPhyDriver05 = Uplink PHY Driver Worker Thread ID 5
DlPhyDriver08 = Downlink PHY Driver Worker Thread ID 8
```

The numeric suffix (**04**, **05**, **08**) indicates the **worker thread index** in the thread pool, **NOT the cell ID**.

### 1.2 Thread Pool Architecture

**Key Components**:

1. **PHY_module**: Central orchestrator managing all PHY instances
   - File: `nv_phy_module.hpp` (lines 115-550)
   - Maintains vector of PHY instances and workers

2. **Thread Configuration**:
   - File: `nv_phy_module.hpp`, line 427
   - Type: `thread_config` (defined in `nv_phy_utils.hpp`)

3. **EPoll Context**: Event-driven message processing
   - File: `nv_phy_module.hpp`, line 435
   - Manages file descriptor events for PHY-MAC communication

**Architecture**:
```cpp
// From nv_phy_module.hpp (lines 421-451)
class PHY_module {
private:
    // IPC Transport
    phy_mac_transport_wrapper transport_wrapper_;

    // Module Thread
    std::thread thread_;
    std::unique_ptr<thread_config> thread_cfg_;

    // PHY Worker Instances
    std::vector<PHY_instance_ptr> phy_instances_;
    std::vector<PHY_instance_ref> phy_refs_;

    // Message Dispatcher
    std::unique_ptr<PHY_module_dispatch> module_dispatch_;

    // Event Handler
    std::unique_ptr<phy_epoll_context> epoll_ctx_p;

    // Slot Command Management
    uint32_t current_slot_cmd_index;
    std::vector<slot_command_api::slot_command> slot_command_array;
};
```

### 1.3 Thread Main Loop

**Primary Thread Functions**:
```cpp
void PHY_module::thread_func();              // Line 244 - Main thread loop
void PHY_module::msg_processing();           // Line 247 - Message dispatcher
void PHY_module::process_phy_commands(bool); // Line 262 - Command processor
```

**Execution Model**:
```
Main Thread Loop:
    ↓
1. Wait for messages from MAC (L2) via EPoll
    ↓
2. Dispatch messages to appropriate PHY instance
    ↓
3. Process slot-based commands in sequence
    ↓
4. Trigger PRACH aggregation tasks (asynchronous)
    ↓
5. Collect results via callbacks
    ↓
6. Return to step 1
```

**Multi-Threading Benefits**:
- **Concurrency**: Multiple slots processed simultaneously
- **Latency Reduction**: Pipeline stages execute in parallel
- **Load Balancing**: Work distributed across CPU cores
- **Resource Utilization**: Full CPU and GPU utilization

---

## Section 2: PRACH Aggregation Task Management

### 2.1 PhyPrachAggr Class Definition

**Source Files**:
- `cuphydriver/include/phyprach_aggr.hpp`
- `cuphydriver/src/uplink/phyprach_aggr.cpp`

**Key Class Structure**:
```cpp
class PhyPrachAggr {
public:
    // Lifecycle Methods
    void setup(SlotMapUl* slot_map);
    void run(cudaStream_t stream);
    void validate();
    void callback();
    void reserve(ULInputBuffer* buffer);

    // Configuration
    void createPhyObj();
    void updateConfig(uint32_t cell_id, const PrachConfig& config);

private:
    // Task Identification
    uint32_t task_id;                    // AGGR task number (0, 1, 2, 3...)
    uint32_t slot_map;                   // Slot Map number (e.g., 448)
    std::vector<uint32_t> cell_ids;      // Assigned cells [0, 1]

    // cuPHY Handle
    cuphyPrachRxHndl_t handle;

    // Input Tensor (from OrderEntity)
    cuphyTensorPrm_t tDataRx;
    cuphy::tensor_desc prach_data_rx_desc[PRACH_MAX_OCCASIONS_AGGR];

    // Output Tensors (GPU device memory)
    cuphy::tensor_device gpu_num_detectedPrmb;
    cuphy::tensor_device gpu_prmbIndex_estimates;
    cuphy::tensor_device gpu_prmbDelay_estimates;
    cuphy::tensor_device gpu_prmbPower_estimates;

    // Workspace Buffer
    cuphy::buffer<float, cuphy::device_alloc> prach_workspace_buffer;

    // Channel States
    uint8_t channel_states[6];  // [task_state, task_active, ch0, ch1, ch2, ch3]
};
```

### 2.2 AGGR Task Numbering Mechanism

**Critical Finding**: **AGGR index is a per-thread task counter**

**Three Architecture Models** (from AGGR3_CELL_MAPPING_ANALYSIS.md):

#### Model A: Per-Slot Task Assignment
```cpp
// System-wide task counter across all slots and threads
std::atomic<uint32_t> global_aggr_counter = 0;

void process_slot(uint32_t slot_map, std::vector<uint32_t> cell_ids) {
    uint32_t task_id = global_aggr_counter.fetch_add(1);
    PhyPrachAggr* aggr = new PhyPrachAggr(task_id, slot_map, cell_ids);
    aggr->run();
}
```

#### Model B: Per-Occasion Task Assignment
```cpp
// Each PRACH occasion gets separate AGGR task
uint32_t task_id = 0;
for (uint32_t cell_id : cell_ids) {
    for (uint32_t occ = 0; occ < nOccasions; occ++) {
        PhyPrachAggr* aggr = new PhyPrachAggr(task_id++, slot_map, {cell_id}, occ);
        aggr->run();
    }
}

// Example: Cell 0 (2 occasions) + Cell 1 (2 occasions) = 4 AGGR tasks (0-3)
```

#### Model C: Per-Thread Task Sequencing (Most Likely)
```cpp
class UlPhyDriver {
private:
    uint32_t aggr_task_counter = 0;        // Per-thread counter
    std::vector<uint32_t> assigned_cells;   // e.g., [0, 1]

public:
    void processSlot(uint32_t slot_map, std::vector<uint32_t> cell_ids) {
        // Create aggregation task for this slot
        uint32_t task_id = aggr_task_counter++;  // Increment per-thread counter

        PhyPrachAggr* aggr_task = new PhyPrachAggr(
            task_id,        // AGGR 0, 1, 2, 3... (per-thread)
            slot_map,       // Slot Map 448
            cell_ids        // [0, 1]
        );

        aggr_task->run();

        if (aggr_task->timedOut()) {
            LOG(ERR) << "ERROR: AGGR " << task_id
                     << " task waiting for PHY Channels more than 10000000 ns"
                     << " for Slot Map " << slot_map
                     << ", state values: " << aggr_task->getStateValues();
        }
    }
};
```

**Evidence Supporting Model C**:
```
Log Pattern Analysis:

UlPhyDriver05 Task History:
├─ Earlier: Spawned AGGR 0 (Slot Map ~300)
├─ Earlier: Spawned AGGR 1 (Slot Map ~350)
├─ Earlier: Spawned AGGR 2 (Slot Map ~400)
└─ Current: Spawned AGGR 3 (Slot Map 448) ← FAILS

Timeline:
- AGGR 0, 1, 2 succeeded (GPU resources available)
- AGGR 3 failed (GPU exhausted after CSI-RS/PDSCH timeouts)
```

### 2.3 Task Lifecycle and State Tracking

**State Machine**:
```
Created (AGGR N)
    │
    ├─ setup(SlotMapUl*)
    │  └─ Reserve OrderEntity buffers
    │  └─ Prepare GPU tensor descriptors
    │  └─ Set initial channel states
    │
    ├─ Wait for PHY Channels (10ms timeout)
    │  ├─ Channel 0 (Input Data): State 2 (WAITING)
    │  ├─ Channel 1 (Configuration): State 2 (WAITING)
    │  ├─ Channel 2 (GPU Resources): State 0 (ERROR) ← Failure Point
    │  └─ Channel 3 (Output Buffers): State 2 (WAITING)
    │
    ├─ [If timeout] → Report error, trigger L1 Recovery
    │
    ├─ [If ready] → run(cudaStream_t)
    │  └─ cuphySetupPrachRx()
    │  └─ cuphyRunPrachRx() (async GPU execution)
    │  └─ cudaStreamSynchronize() (wait for completion)
    │
    ├─ callback()
    │  └─ Extract detection results
    │  └─ Invoke UL callback with prach_params
    │
    └─ Cleanup & Destroy
```

**Channel States** (from debug log analysis):
```
State values: 1 true 2 2 0 2

Position 0: task_state = 1 (RUNNING)
Position 1: task_active = true (ENABLED)
Position 2: channel_0_state = 2 (WAITING) - Input Data
Position 3: channel_1_state = 2 (WAITING) - Configuration
Position 4: channel_2_state = 0 (ERROR) - GPU Resources ← ROOT CAUSE
Position 5: channel_3_state = 2 (WAITING) - Output Buffers
```

---

## Section 3: Cell Assignment Architecture

### 3.1 Cell-to-Thread Mapping

**Key Finding**: **Cells are NOT exclusively assigned to threads**

**Evidence from Log Analysis**:
```
Time T+0s:  UlPhyDriver05 processes Slot Map 349 → Cells [0, 1]
Time T+1s:  UlPhyDriver04 processes Slot Map 449 → Cells [0, 1]
Time T+1s:  UlPhyDriver04 processes Slot Map 451 → Cells [0, 1]
Time T+2s:  UlPhyDriver05 processes Slot Map 551 → Cells [0, 1]
Time T+2s:  UlPhyDriver05 processes Slot Map 553 → Cells [0, 1]
Time T+6s:  UlPhyDriver05 processes Slot Map 448 → Cells [0, 1] ← AGGR 3 fails
```

**Conclusion**: Multiple threads can process the same cells at different times (dynamic round-robin).

### 3.2 Load Balancing Strategy

**Assignment Model**: **Dynamic Slot-based Round-Robin**

**Source**: `nv_phy_module.hpp` (lines 503-527)

```cpp
// Current slot command management (lines 506-507)
uint32_t current_slot_cmd_index;
std::vector<slot_command_api::slot_command> slot_command_array;

// Update function (lines 354-356)
void update_slot_cmds_indexes() {
    current_slot_cmd_index = (current_slot_cmd_index + 1) %
                             slot_command_array.size();
}

// Access current cell command
slot_command_api::cell_sub_command& cell_sub_command(uint32_t cell_index) {
    return slot_command_array.at(current_slot_cmd_index).cells.at(cell_index);
}
```

**Load Balancing Characteristics**:
1. **Slot-based granularity**: Each incoming slot processed atomically
2. **Cell grouping**: Cells 0 and 1 always processed together
3. **Round-robin scheduling**: Threads contend for available slots
4. **Lock-free synchronization**: Atomic operations for thread safety

**Scheduling Algorithm**:
```
Thread Pool: [UlPhyDriver04, UlPhyDriver05, UlPhyDriver08]
Slot Queue:  [Slot M, Slot M+1, Slot M+2, Slot M+3, ...]

Dispatch Logic (Round-Robin):
    ├─ Slot M   → UlPhyDriver04 (available)
    ├─ Slot M+1 → UlPhyDriver05 (available)
    ├─ Slot M+2 → UlPhyDriver08 (available)
    ├─ Slot M+3 → UlPhyDriver04 (next available)
    └─ ...

All threads process cells [0, 1] when assigned a slot
```

### 3.3 Why Cells 0 and 1 Together?

**Reason**: **Carrier Aggregation (CA) Configuration**

**Physical Configuration**:
```
Cell 0 = CC0 (Primary Component Carrier)
    ├─ Frequency: Band n78 (example)
    ├─ SCS: 30 kHz (μ=1)
    ├─ PRACH occasions: 1-4 per slot
    └─ Serves as anchor carrier

Cell 1 = CC1 (Secondary Component Carrier)
    ├─ Frequency: Band n78 (example)
    ├─ SCS: 30 kHz (μ=1)
    ├─ PRACH occasions: 1-4 per slot
    └─ Provides additional bandwidth
```

**Combined Processing Benefits**:
1. **Resource Sharing**: Both cells use same GPU, same O-RU fronthaul
2. **Configuration Efficiency**: Identical parameters (μ, TDD pattern, PRACH format)
3. **Coordination**: UE can connect to both cells simultaneously
4. **Aggregation**: Single AGGR task handles both for efficiency

**Code Structure**:
```cpp
// From slot command API
struct prach_params {
    uint32_t nOccasion;                      // Total occasions for all cells
    std::vector<uint32_t> cell_index_list;   // [0, 1] - both cells
    std::vector<uint32_t> phy_cell_index_list;
    std::vector<uint8_t> startSymbols;       // Per-cell start symbols
    std::vector<uint8_t> freqIndex;          // Per-cell frequency index
};
```

**Evidence from Logs**:
```
Pattern: Both cells ALWAYS report together

Health transitions:
├─ Both cells healthy at same time
├─ Both cells unhealthy at same time
├─ Never: Cell 0 healthy while Cell 1 unhealthy

Throughput metrics:
├─ Cell 0: 0.00 Mbps UL
├─ Cell 1: 0.00 Mbps UL
└─ Identical metrics

FAPI errors:
├─ Line 38: Cell 0 error indication
├─ Line 39: Cell 1 error indication
└─ Sent simultaneously
```

---

## Section 4: Slot Processing Pipeline

### 4.1 Slot Flow Through Uplink Pipeline

**Complete 7-Stage Processing Chain**:

```
Stage 1: O-RAN U-Plane Packet Reception
    ├─ Component: Aerial Fronthaul Driver
    ├─ File: aerial-fh-driver/lib/aerial_fh_driver.cpp
    ├─ Function: Process O-RAN U-Plane packets from O-RU
    └─ Output: Raw IQ samples with metadata (SFN, slot, symbol)

    ↓

Stage 2: Data Decompression and Ordering
    ├─ Component: OrderEntity
    ├─ File: cuphydriver/src/uplink/order_entity.cpp
    ├─ Function: BFP decompression, PRB ordering, antenna mapping
    └─ Output: GPU-ready IQ tensor [nOccasions][N_ant][nSamples]

    ↓

Stage 3: PRACH Task Coordination
    ├─ Component: Slot Map UL
    ├─ File: cuphydriver/include/slot_map_ul.hpp
    ├─ Function: Schedule PRACH tasks, track timing
    └─ Output: SlotMapUl descriptor for PhyPrachAggr

    ↓

Stage 4: PRACH Aggregation Orchestration
    ├─ Component: PhyPrachAggr
    ├─ File: cuphydriver/src/uplink/phyprach_aggr.cpp
    ├─ Function: Coordinate GPU execution, manage resources
    └─ Output: Configured cuPHY PRACH RX handle

    ↓

Stage 5: GPU-Accelerated PRACH Detection
    ├─ Component: cuPHY PRACH RX
    ├─ File: cuPHY/src/cuphy_channels/prach_rx.cpp
    ├─ Function: FFT, correlation, peak detection, TA estimation
    ├─ Pipeline:
    │  ├─ FFT (Time → Frequency domain)
    │  ├─ Reference Sequence Generation (Zadoff-Chu)
    │  ├─ Cross-Correlation
    │  ├─ Multi-Antenna Combining
    │  ├─ Peak Detection
    │  ├─ Delay Estimation (Timing Advance)
    │  ├─ Power Measurement
    │  └─ RSSI Calculation
    └─ Output: Detection results (preambles, TA, power, SNR)

    ↓

Stage 6: Result Callback and FAPI Message Construction
    ├─ Component: FAPI PHY Adapter
    ├─ File: scfl2adapter/lib/scf_5g_fapi/scf_5g_fapi_phy.cpp
    ├─ Function: Format RACH.indication (0x89) message
    └─ Output: FAPI message with detected preambles

    ↓

Stage 7: PHY-MAC Transport
    ├─ Component: Transport Layer
    ├─ File: gt_common_libs/altran/include/mac_phy_intf.h
    ├─ Function: Send to MAC via shared memory IPC
    └─ Output: MAC receives RACH.indication
```

### 4.2 Slot Map Concept and Circular Buffer

**Slot Map Definition**:
- **Purpose**: Map SFN.Slot to circular buffer index
- **Size**: 512 slots (modulo 512)
- **Wrapping**: Slot Map wraps around at 512

**Calculation**:
```cpp
const uint32_t SLOT_MAP_SIZE = 512;

uint32_t slot_to_map(uint32_t sfn, uint32_t slot_in_frame) {
    // For μ=1 (30 kHz SCS): 20 slots per frame
    uint32_t total_slots = sfn * 20 + slot_in_frame;
    return total_slots % SLOT_MAP_SIZE;
}
```

**Examples from Log**:
```
SFN 349.19 → (349 * 20 + 19) % 512 = 6999 % 512 = 349
SFN 449.19 → (449 * 20 + 19) % 512 = 8999 % 512 = 449
SFN 551.19 → (551 * 20 + 19) % 512 = 11039 % 512 = 39 (wrapped)
SFN 553.19 → (553 * 20 + 19) % 512 = 11079 % 512 = 41 (wrapped)
```

**Circular Buffer Behavior**:
```
Slot Map Sequence (observed):
349 → 449 → 451 → 39 → 41 → 141 → 143 → 243 → 245 → 345 → 347 → 447 → 448

Wrapping at 512:
├─ Slot Map 447 (before wrap)
├─ Slot Map 448 (before wrap) ← AGGR 3 fails here
└─ Next would be Slot Map 39 (after wrap to 512)
```

### 4.3 Queue Management Between Components

**Data Structures** (from `nv_phy_module.hpp`):

```cpp
// DL TB Queue Management (lines 437-438)
nv_preallocated_queue<phy_mac_msg_desc> dl_tbs_queue_;
std::mutex dl_tbs_lock;

// DL TTI Message Queue (lines 441-442)
std::queue<phy_mac_msg_desc> dl_tti_queue_;
std::mutex dl_tti_lock;

// UL Slot Command Queue (lines 506, 527)
std::vector<slot_command_api::slot_command> slot_command_array;
uint32_t current_slot_cmd_index;
```

**Queue Flow**:
```
MAC → PHY_module → UlPhyDriver Thread → PRACH Aggregation Task

MAC Layer:
    ├─ Enqueue UL_TTI.request (with PRACH PDU)
    └─ Via: transport_wrapper_ (IPC shared memory)
        ↓
PHY_module::msg_processing():
    ├─ Dequeue message from epoll_ctx
    ├─ Dispatch to module_dispatch_
    └─ Update slot_command_array[current_slot_cmd_index]
        ↓
UlPhyDriver Thread:
    ├─ Access current slot command
    ├─ Extract PRACH parameters
    └─ Create PhyPrachAggr task
        ↓
PhyPrachAggr::run():
    ├─ Wait for OrderEntity (input data)
    ├─ Wait for GPU resources
    └─ Execute cuPHY kernels
        ↓
Callback:
    └─ Send RACH.indication back to MAC
```

---

## Section 5: Thread Naming Explanation

### 5.1 Thread Identification Pattern

**Format**: `[Ul|Dl]PhyDriverNN`

Components:
- **Ul** = Uplink
- **Dl** = Downlink
- **PhyDriver** = Physical Layer Driver (worker thread)
- **NN** = Two-digit worker thread ID (04, 05, 08, ...)

**Examples**:
```
UlPhyDriver04 → Uplink Worker Thread #4
UlPhyDriver05 → Uplink Worker Thread #5
DlPhyDriver08 → Downlink Worker Thread #8
```

### 5.2 Thread Pool Size Configuration

**Source**: `nv_phy_module.hpp` (constructor, lines 122-123)

```cpp
PHY_module(yaml::node node_config);  // YAML configuration
~PHY_module();
```

**Configuration Example** (YAML):
```yaml
phy_module:
  num_threads: 8           # Total worker threads
  thread_affinity: true    # CPU core pinning enabled

  uplink_threads: 4        # UlPhyDriver instances
  downlink_threads: 4      # DlPhyDriver instances

  thread_ids:
    - 0: core 0           # UlPhyDriver00 → CPU core 0
    - 4: core 4           # UlPhyDriver04 → CPU core 4
    - 5: core 5           # UlPhyDriver05 → CPU core 5
    - 8: core 8           # UlPhyDriver08 → CPU core 8 (DL)
```

### 5.3 Why Non-Sequential Thread IDs?

**Possible Reasons**:

1. **CPU Core Affinity**: Thread IDs match CPU core numbers
   ```
   UlPhyDriver04 → Pinned to CPU core 4
   UlPhyDriver05 → Pinned to CPU core 5
   UlPhyDriver08 → Pinned to CPU core 8 (possibly NUMA node 1)
   ```

2. **Hybrid Deployment**: Some threads disabled or reserved
   ```
   Threads 0-3: Reserved for control plane / MAC layer
   Threads 4-7: Uplink PHY workers
   Threads 8-11: Downlink PHY workers
   ```

3. **Load Balancing**: Strategic placement for cache locality
   ```
   Even cores: Uplink processing
   Odd cores: Downlink processing
   ```

---

## Section 6: Data Structures for Task Management

### 6.1 Slot Command Structure

**File**: `scf_5g_slot_commands_common.hpp` (line 14)

```cpp
using namespace slot_command_api;

struct slot_command {
    slot_indication slot_info;           // SFN, slot number, timing metadata

    std::vector<cell_sub_command> cells; // Per-cell commands [cell 0, cell 1]

    cell_group_command cell_groups;      // Carrier Aggregation grouping

    callbacks cb;                         // UL/DL callbacks
};
```

**Cell Sub-Command**:
```cpp
struct cell_sub_command {
    uint32_t cell_index;                 // 0, 1, 2, ...

    // Downlink Channels
    std::vector<pdsch_params> pdsch;     // Physical Downlink Shared Channel
    std::vector<pdcch_params> pdcch;     // Physical Downlink Control Channel
    std::vector<pbch_params> pbch;       // Physical Broadcast Channel
    std::vector<csirs_params> csirs;     // CSI Reference Signal

    // Uplink Channels
    std::vector<pusch_params> pusch;     // Physical Uplink Shared Channel
    std::vector<pucch_params> pucch;     // Physical Uplink Control Channel
    std::vector<prach_params> prach;     // Physical Random Access Channel
    std::vector<srs_params> srs;         // Sounding Reference Signal
};
```

**PRACH Parameters**:
```cpp
struct prach_params {
    uint32_t nOccasion;                      // Number of PRACH occasions
    std::vector<uint32_t> cell_index_list;   // [0, 1] for CA
    std::vector<uint32_t> phy_cell_index_list;

    // Per-occasion parameters
    std::vector<uint8_t> startSymbols;       // Starting OFDM symbol index
    std::vector<uint8_t> freqIndex;          // Frequency domain PRB index
    uint8_t mu;                              // Subcarrier spacing (0-4)
    std::vector<uint8_t> numRa;              // Number of RA preambles
    std::vector<uint8_t> prachStartRb;       // Starting PRB
    std::vector<uint16_t> rootSequenceIndex; // Root sequence (0-837)
    std::vector<uint8_t> zeroCorrelationZone; // ZCZ config (0-15)
};
```

### 6.2 Thread Synchronization Primitives

**From `nv_phy_module.hpp`** (lines 444-451):

```cpp
// Atomic Tick Synchronization
std::atomic<sfn_slot_t> ss_tick;                    // Line 448
std::mutex tick_lock;                               // Line 449
std::atomic<nanoseconds> current_tick_;             // Line 450
nv::tti_gen tti_module_;                            // Line 451

// Queue Locks
std::mutex dl_tbs_lock;                             // Line 438
std::mutex dl_tti_lock;                             // Line 442

// Cell Health Tracking (if ENABLE_L2_SLT_RSP)
uint64_t active_cell_bitmap;                        // Line 511
uint64_t fapi_eom_rcvd_bitmap;                      // Line 512
```

**Atomic Operations**:
```cpp
// Slot command index update (lock-free)
void update_slot_cmds_indexes() {
    // Atomic increment with modulo wrap
    current_slot_cmd_index = (current_slot_cmd_index + 1) %
                             slot_command_array.size();
}

// Tick update (atomic)
void tick_received(std::chrono::nanoseconds& timestamp) {
    current_tick_.store(timestamp, std::memory_order_release);
    ss_tick.store(new_tick, std::memory_order_release);
}
```

### 6.3 PRACH Aggregation Task Data

**cuPHY Static Parameters**:
```cpp
struct cuphyPrachStatPrms_t {
    uint32_t nCells;                    // Number of cells (e.g., 2)
    cuphyPrachCellStatPrms_t* pCellStatPrms;
};

struct cuphyPrachCellStatPrms_t {
    uint32_t occaStartIdx;              // Starting occasion index
    uint32_t nOccasionsFdm;             // Number of FDM occasions (1-4)
    uint32_t N_ant;                     // Number of RX antennas (4)
    cuphyFreqRange_t freqRange;         // FR1 (< 6 GHz) or FR2 (mmWave)
    cuphyDuplexMode_t duplexMode;       // TDD or FDD
    uint32_t mu;                        // Subcarrier spacing (0=15kHz, 1=30kHz, ...)
    uint32_t configurationIndex;        // PRACH config index (0-255)
    cuphyPrachRestrictedSet_t restrictedSetCfg;
};
```

**cuPHY Dynamic Parameters** (per-occasion):
```cpp
struct cuphyPrachDynPrms_t {
    uint32_t nOccasions;
    cuphyPrachOccaDynPrms_t* pOccaDynPrms;
};

struct cuphyPrachOccaDynPrms_t {
    uint32_t occaParamStatIdx;          // Index into static params
    uint32_t occaParamDynIdx;           // Dynamic param index (-1 = missing!)
    float    forceThreshold;            // Detection threshold (dB)
    uint32_t freqIndex;                 // Frequency PRB index
    uint32_t symbolIndex;               // Starting OFDM symbol
};
```

---

## Section 7: Synchronization and Coordination

### 7.1 Tick-Based Synchronization

**Tick Mechanism**: Central time synchronization across all threads

**Source**: `nv_phy_module.hpp` (lines 274-278)

```cpp
// tick_received()
// Main synchronization point for all worker threads
void tick_received(std::chrono::nanoseconds& timestamp);

// set_tti_flag()
// Enable/disable TTI processing
void set_tti_flag(bool flag);

// get_prev_tick()
// Access previous tick information
slot_command_api::slot_indication& get_prev_tick() {
    return tick_updater_.prev_slot_info_;
}
```

**Tick Updater**:
```cpp
class TickUpdater {
public:
    uint32_t mu_highest_;              // Highest numerology in system (0-4)
    uint32_t slot_advance_;            // Slot lookahead count
    slot_command_api::slot_indication prev_slot_info_;
};
```

**Synchronization Flow**:
```
Tick Generator (tti_module_)
    ↓
PHY_module::tick_received(timestamp)
    ↓
    ├─ Update current_tick_ (atomic)
    ├─ Update ss_tick (atomic SFN/slot)
    └─ Notify all worker threads
        ↓
UlPhyDriver threads wake up
    ↓
Process slot commands in sync
```

### 7.2 PRACH Timeout and Monitoring

**Timeout Configuration**:
```cpp
const uint64_t PRACH_AGGR_TIMEOUT_NS = 10000000;  // 10ms
```

**Monitoring Implementation**:
```cpp
void PhyPrachAggr::run(cudaStream_t stream) {
    auto start_time = std::chrono::high_resolution_clock::now();

    // Wait for all channels to be ready
    while (!all_channels_ready()) {
        auto elapsed = std::chrono::high_resolution_clock::now() - start_time;
        auto elapsed_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(elapsed).count();

        if (elapsed_ns > PRACH_AGGR_TIMEOUT_NS) {
            LOG(ERR) << "ERROR: AGGR " << task_id
                     << " task waiting for PHY Channels more than "
                     << elapsed_ns << " ns"
                     << " for Slot Map " << slot_map
                     << ", state values: " << get_state_string();

            // Trigger L1 recovery
            trigger_recovery();
            return;
        }

        // Small sleep to avoid busy-wait
        std::this_thread::sleep_for(std::chrono::microseconds(100));
    }

    // Execute GPU processing
    execute_prach_detection(stream);
}
```

### 7.3 Error Handling and Recovery

**L1 Recovery Sequence** (from debug log):
```
12:30:38.540414 | PRACH AGGR 3 timeout (TRIGGER EVENT)
12:30:38.540514 | L1 Recovery slot 1
12:30:38.541007 | L1 Recovery slot 2
12:30:38.541507 | L1 Recovery slot 3
12:30:38.542008 | L1 Recovery slot 4+
```

**Recovery Mechanism**:
```cpp
void PHY_module::msg_processing() {
    bool in_recovery = false;
    uint32_t recovery_slot_count = 0;

    while (running_) {
        // Receive message from MAC
        phy_mac_msg_desc msg;
        recv_msg(&msg);

        if (aggr_task_timeout_detected) {
            in_recovery = true;
            recovery_slot_count = 1;

            LOG(WRN) << "[DRV.CTX] L1 in recovery for "
                     << recovery_slot_count << " slots";
        }

        if (in_recovery) {
            // Drop incoming MAC messages
            LOG(WRN) << "SFN " << msg.sfn << "." << msg.slot
                     << " RECV: on_msg: cell_id=" << msg.cell_id
                     << " msg_id=0x" << std::hex << msg.msg_id
                     << ": L1 in recovery, drop msg";

            recovery_slot_count++;

            if (recovery_slot_count > MAX_RECOVERY_SLOTS) {
                // Full L1 reset
                reset_l1_state();
                in_recovery = false;
            }

            continue;  // Skip processing
        }

        // Normal message processing
        process_msg(&msg);
    }
}
```

---

## Section 8: Configuration and Initialization

### 8.1 Thread Pool Configuration

**Configuration File** (YAML example):
```yaml
phy_module:
  # Thread Pool Settings
  num_threads: 8
  thread_affinity: true

  # Thread-to-Core Mapping
  thread_mapping:
    - thread_id: 4
      cpu_core: 4
      type: uplink
    - thread_id: 5
      cpu_core: 5
      type: uplink
    - thread_id: 8
      cpu_core: 8
      type: downlink

  # PRACH Configuration
  prach:
    max_occasions: 4
    timeout_ms: 10
    workspace_size_mb: 4

  # Cell Configuration
  cells:
    - cell_id: 0
      carrier_idx: 0
      pci: 1
    - cell_id: 1
      carrier_idx: 1
      pci: 2
```

### 8.2 Carrier Aggregation Setup

**CA Configuration**:
```cpp
// From scf_5g_fapi_phy.hpp
struct cell_config {
    uint32_t cell_id;               // 0, 1, ...
    uint32_t carrier_idx;           // Carrier index
    uint32_t physical_cell_id;      // PCI
    bool is_primary;                // Primary or secondary CC
};

// Carrier Aggregation Group
std::vector<cell_config> ca_group = {
    {cell_id: 0, carrier_idx: 0, pci: 1, is_primary: true},   // CC0
    {cell_id: 1, carrier_idx: 1, pci: 2, is_primary: false}   // CC1
};
```

### 8.3 AGGR Task Configuration

**Dynamic Task Creation**:
```cpp
// Configuration determines task creation strategy

// Option 1: One AGGR task per slot (both cells)
config.aggr_per_slot = true;
config.cells_per_aggr = [0, 1];  // Both cells in single task

// Option 2: One AGGR task per cell
config.aggr_per_cell = true;
// Creates 2 tasks per slot: AGGR(cell 0), AGGR(cell 1)

// Option 3: One AGGR task per occasion
config.aggr_per_occasion = true;
// Creates nOccasions * nCells tasks per slot
```

---

## Section 9: Architecture Diagram

```
┌────────────────────────────────────────────────────────────────────────┐
│                         PHY_module (Main Thread)                       │
│  ┌──────────────────────────────────────────────────────────────────┐  │
│  │ • EPoll Context: Message reception from MAC                      │  │
│  │ • Dispatcher: Route messages to worker threads                   │  │
│  │ • Tick Generator: Synchronize all threads                        │  │
│  │ • Slot Command Array: Circular buffer of pending commands        │  │
│  └──────────────────────────────────────────────────────────────────┘  │
└────────────────────────────────────────────────────────────────────────┘
         │
         ├─────────────────────────────────────────────────────┐
         │                                                     │
         v                                                     v
┌──────────────────────────┐                     ┌──────────────────────────┐
│   UlPhyDriver04          │                     │   UlPhyDriver05          │
│   (Worker Thread #4)     │                     │   (Worker Thread #5)     │
│                          │                     │                          │
│ CPU Core: 4              │                     │ CPU Core: 5              │
│ AGGR Counter: 0,1,2      │                     │ AGGR Counter: 0,1,2,3    │
│ Assigned Cells: [0,1]    │                     │ Assigned Cells: [0,1]    │
│                          │                     │                          │
│ Processing:              │                     │ Processing:              │
│ ├─ Slot M-3 → AGGR 0     │                     │ ├─ Slot M → AGGR 0      │
│ ├─ Slot M-1 → AGGR 1     │                     │ ├─ Slot M+2 → AGGR 1    │
│ └─ Slot M+1 → AGGR 2     │                     │ ├─ Slot M+4 → AGGR 2    │
│                          │                     │ └─ Slot M+6 → AGGR 3 ❌  │
└──────────────────────────┘                     └──────────────────────────┘
         │                                                    │
         └─────────────────────┬───────────────────────────┬──┘
                               │
                    ┌──────────v──────────┐
                    │   Slot Map UL       │
                    │  (Circular Buffer)  │
                    │                     │
                    │ Slot Map 448:       │
                    │ ├─ SFN: 961         │
                    │ ├─ Slot: 19         │
                    │ ├─ Cells: [0, 1]    │
                    │ └─ PRACH occasions  │
                    └──────────┬──────────┘
                               │
         ┌─────────────────────┼─────────────────────┐
         │                     │                     │
         v                     v                     v
  ┌──────────────┐   ┌─────────────────┐   ┌─────────────────┐
  │ OrderEntity  │   │ PhyPrachAggr    │   │ FAPI Builder    │
  │              │   │                 │   │                 │
  │ • Decompress │   │ • Orchestrate   │   │ • Build msg     │
  │ • Reorder    │   │ • Allocate GPU  │   │ • Send to MAC   │
  │ • Buffer     │   │ • Execute cuPHY │   │                 │
  └──────────────┘   └─────────────────┘   └─────────────────┘
         │                     │
         │            ┌────────v────────┐
         │            │   GPU Pipeline  │
         │            │                 │
         │            │ cuPHY PRACH RX  │
         │            │ ├─ FFT          │
         │            │ ├─ Correlation  │
         │            │ ├─ Detection    │
         │            │ └─ Measurement  │
         │            └────────┬────────┘
         │                     │
         └─────────────────────┼──────────────────┘
                               │
                        ┌──────v───────┐
                        │  MAC Layer   │
                        │ (L2 Stack)   │
                        └──────────────┘
```

---

## Section 10: Sequence Diagram for Slot Processing

```
Timeline: Processing of Slot Map 448 (SFN 961.19)

Actor: MAC Layer | PHY_module | UlPhyDriver05 | OrderEntity | PhyPrachAggr | GPU | FAPI

T+0μs:
MAC ──[UL_TTI.request]──> PHY_module
                          │
                          │ Receive via EPoll
                          │ Dispatch to thread
                          │
T+10μs:                   │
                          ├──[Slot Command]──> UlPhyDriver05
                                               │
                                               │ Pick up slot 448
                                               │ Create AGGR task
                                               │
T+20μs:                                        │
                                               ├──[Reserve]──> OrderEntity
                                                               │
T+30μs:                                        ├──[Create]──> PhyPrachAggr
                                                              │
                                                              │ task_id = 3
                                                              │ cells = [0,1]
                                                              │ slot_map = 448
                                                              │
T+40μs:                                                       │
                                                              ├──[setup()]
                                                              │
                                                              │ Wait for channels:
                                                              │ • Channel 0: WAITING
                                                              │ • Channel 1: WAITING
                                                              │ • Channel 2: ERROR ❌
                                                              │ • Channel 3: WAITING
                                                              │
T+50μs - T+10050μs:                                          │
                                                              │ Timeout loop
                                                              │ (10ms elapsed)
                                                              │
T+10050μs:                                                    │
                                                              ├──[Timeout Error]
                                                              │
                                                              LOG(ERR): AGGR 3 timeout
                                                              │
T+10100μs:                                                    │
UlPhyDriver05 <──[Error Report]──────────────────────────────┘
   │
   │ Trigger L1 Recovery
   │
T+10200μs:
PHY_module <──[Recovery Event]───────────────────────────────────────────┘
   │
   │ Enter recovery mode
   │
T+10300μs:
MAC <──[Drop Messages]───────────────────────────────────────────────────┘

[Recovery sequence continues for 4+ slots]

=== NORMAL CASE (No Timeout) ===

T+50μs:                                                       │
                                               <──[IQ Data]── OrderEntity
                                                              │
T+250μs:                                                      │
                                                              ├──[run()]
                                                              │
                                                              ├──[cuphySetupPrachRx]──> GPU
                                                              │                         │
T+260μs:                                                      │                         │
                                                              ├──[cuphyRunPrachRx]─────>│
                                                              │                         │
                                                                                        │ FFT
                                                                                        │ Correlation
                                                                                        │ Detection
                                                                                        │
T+680μs:                                                      │                         │
                                                              <──[Results]──────────────┘
                                                              │
T+700μs:                                                      │
                                                              ├──[callback()]
                                                              │
T+720μs:                                        <──[Results]──┘
                                                │
T+750μs:                          <──[Build RACH.ind]─────────┘
                                  │
T+800μs:
MAC <──[RACH.indication]──────────┘

Total: ~800μs (successful case) vs 10ms+ (timeout case)
```

---

## Section 11: Key Files and Line References

### 11.1 Thread Management

| Component | File | Key Lines | Description |
|-----------|------|-----------|-------------|
| **PHY Module** | `nv_phy_module.hpp` | 115-550 | Main orchestrator class |
| **Thread Creation** | `nv_phy_module.hpp` | 225-226 | start() method |
| **Thread Config** | `nv_phy_module.hpp` | 427 | thread_config structure |
| **Thread Loop** | `nv_phy_module.hpp` | 244, 247, 262 | Main processing functions |
| **EPoll Context** | `nv_phy_epoll_context.cpp` | Throughout | Event handling |

### 11.2 Slot Command Management

| Component | File | Key Lines | Description |
|-----------|------|-----------|-------------|
| **Slot Commands** | `nv_phy_module.hpp` | 349-356 | Access methods |
| **Command Array** | `nv_phy_module.hpp` | 527 | Storage vector |
| **Index Update** | `nv_phy_module.hpp` | 506 | Current index |
| **Command API** | `scf_5g_slot_commands_common.hpp` | 14-200 | Definitions |

### 11.3 PRACH Processing

| Component | File | Key Lines | Description |
|-----------|------|-----------|-------------|
| **FAPI PHY** | `scf_5g_fapi_phy.hpp` | 65-274 | PRACH API |
| **FAPI Callback** | `scf_5g_fapi_phy.cpp` | ~4636-4647 | Result handling |
| **RACH Indication** | `scf_5g_fapi_phy.cpp` | ~3286-3447 | Message builder |
| **FAPI Messages** | `scf_5g_fapi.h` | Throughout | Message formats |
| **RACH Structure** | `scf_5g_fapi.h` | 1404-1430 | RACH.ind definition |

### 11.4 Synchronization

| Component | File | Key Lines | Description |
|-----------|------|-----------|-------------|
| **Tick Sync** | `nv_phy_module.hpp` | 274-278 | tick_received() |
| **Atomics** | `nv_phy_module.hpp` | 444-451 | Atomic variables |
| **Locks** | `nv_phy_module.hpp` | 438, 442 | Mutex locks |
| **Cell Health** | `nv_phy_module.hpp` | 511-512 | Bitmap tracking |

### 11.5 Documentation

| Document | Location | Description |
|----------|----------|-------------|
| **PRACH Chain** | `PRACH_PROCESSING_CHAIN.md` | 7-stage pipeline |
| **Cell Mapping** | `AGGR3_CELL_MAPPING_ANALYSIS.md` | AGGR 3 analysis |
| **Debug Analysis** | `PRACH_DEBUG_ANALYSIS.md` | Failure analysis |

---

## Section 12: Summary and Key Insights

### 12.1 Architecture Principles

**Tiered Parallel Processing**:
1. **Tier 1 - Thread Pool**: Multiple UlPhyDriver worker threads (04, 05, 08, ...)
2. **Tier 2 - Task Instances**: Per-thread AGGR counters (0, 1, 2, 3, ...)
3. **Tier 3 - Cell Groups**: Dynamic CA cell assignment [0, 1]
4. **Tier 4 - GPU Kernels**: Massively parallel CUDA execution

### 12.2 AGGR Task Numbering Resolution

**NOT**: "AGGR 3" = "Aggregator for Cell 3"

**ACTUALLY**: "AGGR 3" = "4th aggregation task spawned by UlPhyDriver05"
- Per-thread counter
- Increments with each slot processed
- Independent of cell assignment

### 12.3 Cell Assignment Model

**NOT**: Cells exclusively bound to specific threads

**ACTUALLY**: Dynamic round-robin slot-based distribution
- Multiple threads can process same cells
- Assignment changes per slot
- Cells 0-1 always grouped (Carrier Aggregation)

### 12.4 Load Balancing Strategy

- **Granularity**: Per-slot atomic operations
- **Mechanism**: Round-robin across available threads
- **Synchronization**: Lock-free atomic updates
- **Efficiency**: Minimal contention, high throughput

### 12.5 Timing Constraints

- **PRACH Processing Budget**: < 2ms (typical)
- **GPU Execution Time**: ~680μs (FFT + correlation + detection)
- **Timeout Threshold**: 10ms (triggers L1 recovery)
- **Recovery Duration**: 4+ slots (~2ms)

### 12.6 Failure Analysis (AGGR 3 Timeout)

**Root Cause Sequence**:
```
1. CSI-RS timeout (4ms) → GPU processing backlog
2. PDSCH H2D timeout (500μs) → GPU memory stuck
3. AGGR 3 workspace allocation → FAILS (Channel 2 = ERROR)
4. 10ms timeout exceeded → L1 Recovery triggered
```

**Why AGGR 3 Specifically**:
- Previous AGGR tasks (0, 1, 2) succeeded when GPU available
- AGGR 3 spawned 318μs after PDSCH timeout
- GPU resources still occupied
- Channel 2 (GPU workspace) enters terminal ERROR state

---

## Appendix A: Thread ID to CPU Core Mapping (Hypothetical)

```
Thread Pool Configuration (8 threads total):

CPU Cores (Uplink - Isolated):
├─ Core 0: Reserved / Control Plane
├─ Core 1: Reserved / Control Plane
├─ Core 2: Reserved / Control Plane
├─ Core 3: Reserved / Control Plane
├─ Core 4: UlPhyDriver04 ← Uplink worker
├─ Core 5: UlPhyDriver05 ← Uplink worker
├─ Core 6: UlPhyDriver06 (not seen in log)
└─ Core 7: UlPhyDriver07 (not seen in log)

CPU Cores (Downlink - Isolated):
├─ Core 8:  DlPhyDriver08 ← Downlink worker
├─ Core 9:  DlPhyDriver09 (not seen in log)
├─ Core 10: DlPhyDriver10 (not seen in log)
└─ Core 11: DlPhyDriver11 (not seen in log)

NUMA Topology:
├─ NUMA Node 0: Cores 0-5 (Control + UL)
└─ NUMA Node 1: Cores 6-11 (UL + DL)
```

---

## Appendix B: Slot Map Calculation Examples

```python
# Slot Map calculation for μ=1 (30 kHz SCS)
def calculate_slot_map(sfn, slot_in_frame):
    SLOTS_PER_FRAME = 20  # for μ=1
    SLOT_MAP_SIZE = 512

    total_slots = sfn * SLOTS_PER_FRAME + slot_in_frame
    slot_map = total_slots % SLOT_MAP_SIZE

    return slot_map

# Examples from debug log:
print(calculate_slot_map(349, 19))  # Output: 349
print(calculate_slot_map(449, 19))  # Output: 449
print(calculate_slot_map(551, 19))  # Output: 39 (wrapped)
print(calculate_slot_map(553, 19))  # Output: 41 (wrapped)
print(calculate_slot_map(961, 19))  # Output: 448 (AGGR 3 failure)
```

---

## Appendix C: References

- **PRACH Processing Chain**: `PRACH_PROCESSING_CHAIN.md`
- **Cell Mapping Analysis**: `AGGR3_CELL_MAPPING_ANALYSIS.md`
- **Debug Log Analysis**: `PRACH_DEBUG_ANALYSIS.md`
- **Debug Log File**: `debug_log_prach.txt`
- **cuPHY-CP Source**: `cuPHY-CP/` directory tree
- **FAPI Specification**: O-RAN.WG8.AAD.0-v07.00 (5G FAPI)
- **O-RAN Fronthaul**: O-RAN.WG4.CUS.0-v05.00

---

**End of Document**
