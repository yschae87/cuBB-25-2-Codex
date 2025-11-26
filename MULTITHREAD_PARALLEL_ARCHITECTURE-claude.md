# NVIDIA Aerial cuBB Multi-Task Multi-Threaded Parallel Processing Architecture

## Executive Summary

The NVIDIA Aerial cuBB (Cloud-native Base Band) implements a sophisticated multi-tier parallel processing architecture designed for 5G/6G real-time signal processing. This architecture leverages CPU thread pools, GPU CUDA kernels, lock-free synchronization primitives, and event-driven I/O to achieve ultra-low latency (sub-millisecond) processing with high throughput.

### Key Architecture Components
- **cuPHY-CP**: Control Plane PHY - Thread pool management, message routing, timing synchronization
- **cuPHY**: Data Plane PHY - GPU-accelerated physical layer processing (FFT, correlation, detection)
- **cuMAC-CP**: Control Plane MAC - Scheduling coordination, configuration management
- **cuMAC**: Data Plane MAC - Multi-cell scheduling, beamforming, resource allocation

## Table of Contents

1. [System Overview](#1-system-overview)
2. [Directory Structure and Organization](#2-directory-structure-and-organization)
3. [Threading Architecture](#3-threading-architecture)
4. [GPU Parallel Processing](#4-gpu-parallel-processing)
5. [Synchronization Mechanisms](#5-synchronization-mechanisms)
6. [Control Plane vs Data Plane](#6-control-plane-vs-data-plane)
7. [Performance Optimization](#7-performance-optimization)
8. [Configuration and Deployment](#8-configuration-and-deployment)
9. [Key Implementation Files](#9-key-implementation-files)
10. [Architecture Diagrams](#10-architecture-diagrams)

---

## 1. System Overview

### 1.1 Multi-Tier Parallelism

The Aerial cuBB architecture implements parallelism at four distinct tiers:

| Tier | Component | Parallelism Type | Scale |
|------|-----------|-----------------|--------|
| **Tier 1** | CPU Thread Pool | Thread-level parallelism | 8-16 threads |
| **Tier 2** | Task Instances | Task-level parallelism | Per-thread counters |
| **Tier 3** | Cell Processing | Data parallelism | Multiple cells/carriers |
| **Tier 4** | GPU Kernels | Massive parallelism | 1024+ threads/block |

### 1.2 Processing Pipeline

```
MAC Layer (L2)
    ↓ [FAPI Messages]
PHY-MAC Transport (IPC)
    ↓ [EPoll Notification]
PHY_module (Main Thread)
    ↓ [Round-Robin Dispatch]
Worker Threads (UlPhyDriver04/05/08)
    ↓ [Task Creation]
GPU Processing (CUDA Kernels)
    ↓ [Results]
Callback & Response
```

### 1.3 Timing Constraints

| Operation | Budget | Typical | Maximum |
|-----------|--------|---------|---------|
| PRACH Detection | 2ms | 680μs | 10ms (timeout) |
| Slot Processing | 500μs | 350μs | 500μs |
| L1 Recovery | - | 2ms | 4 slots |
| GPU Kernel | - | 50-200μs | 500μs |

---

## 2. Directory Structure and Organization

### 2.1 cuPHY-CP (Control Plane PHY)

```
cuPHY-CP/
├── cuphyl2adapter/         # L2-PHY Interface Layer
│   └── lib/nvPHY/
│       ├── nv_phy_module.hpp/cpp      # Main orchestrator & thread pool
│       ├── nv_phy_instance.hpp        # PHY worker instances
│       ├── nv_phy_driver_proxy.hpp    # Singleton proxy pattern
│       ├── nv_phy_epoll_context.hpp   # Event-driven I/O
│       ├── nv_phy_mac_transport.hpp   # IPC transport
│       └── nv_tick_generator.cpp      # Timing synchronization
├── cuphycontroller/        # PHY Driver Controller
│   ├── cuphydriver.hpp     # Driver API
│   └── yamlparser.hpp      # Configuration parsing
└── aerial-fh-driver/       # O-RAN Fronthaul
    └── IQ reception & decompression
```

### 2.2 cuMAC-CP (Control Plane MAC)

```
cuMAC-CP/
└── lib/
    └── cumac_msg.h         # Message definitions
        ├── Task types (UE_SELECTION, PRB_ALLOCATION, etc.)
        ├── Message formats (CONFIG.req/resp, START.req/resp)
        └── TTI messages (SCH_TTI.request/response)
```

### 2.3 cuPHY (Data Plane PHY)

```
cuPHY/
└── src/
    └── cuphy_channels/
        ├── prach_rx.cpp    # PRACH reception
        ├── pusch_rx.cpp    # PUSCH decoding
        ├── pucch_rx.cpp    # PUCCH decoding
        └── GPU kernels (FFT, correlation, detection)
```

### 2.4 cuMAC (Data Plane MAC)

```
cuMAC/
└── src/
    ├── multiCellScheduler.cu      # Multi-cell scheduling
    ├── multiCellBeamform.cu       # Beamforming operations
    ├── multiCellSinrCal.cu        # SINR calculation
    ├── mcsSelectionLUT.cu         # MCS selection lookup
    ├── multiCellLayerSel.cu       # Layer selection
    └── cumacSubcontext.h          # GPU/CPU execution management
```

---

## 3. Threading Architecture

### 3.1 Thread Pool Configuration

#### Thread Types and Responsibilities

| Thread Name | Core | Type | Responsibilities |
|------------|------|------|-----------------|
| PHY_module | 0-3 | Control | Main orchestrator, message routing |
| UlPhyDriver04 | 4 | Uplink | PRACH/PUSCH processing, AGGR tasks |
| UlPhyDriver05 | 5 | Uplink | PRACH/PUSCH processing, AGGR tasks |
| UlPhyDriver06 | 6 | Uplink | Additional uplink capacity |
| UlPhyDriver07 | 7 | Uplink | Additional uplink capacity |
| DlPhyDriver08 | 8 | Downlink | PDSCH processing |
| DlPhyDriver09-11 | 9-11 | Downlink | Additional downlink capacity |

#### Thread Naming Convention
```cpp
// Format: [UL|DL]PhyDriverNN
// UL/DL: Uplink or Downlink
// NN: Worker thread index (NOT cell ID)
// Example: UlPhyDriver05 = Uplink worker thread #5
```

### 3.2 Thread Pool Implementation

```cpp
// File: nv_phy_module.hpp
class PHY_module {
private:
    // Thread management
    std::thread thread_;                           // Module thread
    std::vector<PHY_instance_ptr> phy_instances_;  // Worker instances
    std::vector<PHY_instance_ref> phy_refs_;       // Worker references

    // Synchronization
    std::atomic<sfn_slot_t> ss_tick;              // Slot/frame number
    std::mutex tick_lock;                          // Tick synchronization
    std::atomic<nanoseconds> current_tick_;        // Current timestamp

    // Message queues
    nv_preallocated_queue<phy_mac_msg_desc> dl_tbs_queue_;
    std::queue<phy_mac_msg_desc> dl_tti_queue_;
    std::mutex dl_tbs_lock, dl_tti_lock;

    // Slot command management
    std::vector<slot_command_api::slot_command> slot_command_array;
    uint32_t current_slot_cmd_index;

public:
    void start() {
        thread_ = std::thread(&PHY_module::thread_func, this);
    }

    void thread_func() {
        while (running) {
            epoll_ctx_p->wait_for_events();  // Event-driven I/O
            process_messages();               // Handle MAC messages
            dispatch_to_workers();            // Round-robin dispatch
        }
    }
};
```

### 3.3 PRACH Aggregation Task Management

#### Task Identification System
```
AGGR N = "Nth aggregation task spawned by worker thread"
- Per-thread task counter (0, 1, 2, 3, ...)
- Independent of cell assignment
- Example: UlPhyDriver05 spawns AGGR 0, AGGR 1, AGGR 2, AGGR 3
```

#### Task State Machine
```
CREATED → SETUP → WAITING → RUNNING → CALLBACK → CLEANUP

States:
- CREATED:  Task object allocated
- SETUP:    Parameters configured
- WAITING:  Waiting for input channels
- RUNNING:  GPU kernel execution
- CALLBACK: Results processing
- CLEANUP:  Resource deallocation
```

#### Task Channels
```cpp
struct PhyPrachAggr {
    // Four input channels
    Channel ch0;  // IQ samples from OrderEntity
    Channel ch1;  // PRACH configuration parameters
    Channel ch2;  // GPU resources (CUDA memory/streams)
    Channel ch3;  // Output buffers for detection results

    // Timeout monitoring
    const uint64_t PRACH_AGGR_TIMEOUT_NS = 10000000;  // 10ms
};
```

### 3.4 Dynamic Cell Assignment

#### Round-Robin Slot Distribution
```
Available Workers: [UlPhyDriver04, UlPhyDriver05, UlPhyDriver08]
Slot Queue: [Slot M, Slot M+1, Slot M+2, ...]

Assignment:
├─ Slot M   → UlPhyDriver04 (processes cells 0,1)
├─ Slot M+1 → UlPhyDriver05 (processes cells 0,1)
├─ Slot M+2 → UlPhyDriver08 (processes cells 0,1)
├─ Slot M+3 → UlPhyDriver04 (next available)
└─ ...

Key Finding: All threads process same cells [0,1] at different times
```

#### Carrier Aggregation Configuration
```cpp
struct prach_params {
    uint32_t nOccasion;
    std::vector<uint32_t> cell_index_list;  // Always [0, 1] for CA
    std::vector<uint8_t> startSymbols;
    std::vector<uint8_t> freqIndex;
    uint32_t mu;                            // Subcarrier spacing
    std::vector<uint8_t> numRa;
    std::vector<uint8_t> prachStartRb;
    std::vector<uint16_t> rootSequenceIndex;
    std::vector<uint8_t> zeroCorrelationZone;
};
```

---

## 4. GPU Parallel Processing

### 4.1 CUDA Kernel Architecture

#### Kernel Launch Configuration
```cuda
// File: multiCellScheduler.cu
__global__ __launch_bounds__(1024, MinBlkPerSM_)
void multiCellSchedulerKernel_svdMmseIrc(
    cumac::mcDynDescr_t* pDynDescr
) {
    // Thread-per-UE model
    uint16_t assocUeIdxInBlk = floor(
        static_cast<float>(threadIdx.x)/nBsAntSqrd
    );
    uint16_t eIdx = threadIdx.x - assocUeIdxInBlk*nBsAntSqrd;

    // Matrix indexing for beamforming
    colIdx[threadIdx.x] = floor(
        static_cast<float>(eIdx)/pDynDescr->nUeAnt
    );
    rowIdx[threadIdx.x] = eIdx - colIdx[threadIdx.x]*pDynDescr->nUeAnt;
    CMatIdx[threadIdx.x] = assocUeIdxInBlk*nUeAntSqrd + eIdx;

    // Warp-level reduction for SINR calculation
    __syncwarp();
    // ... reduction operations ...
}
```

#### Kernel Types and Functions

| Kernel | Function | Threads/Block | Purpose |
|--------|----------|---------------|---------|
| PRACH Detection | FFT + Correlation | 256-512 | Preamble detection |
| Multi-Cell Scheduler | SVD/MMSE | 1024 | Resource allocation |
| Beamforming | Matrix operations | 512 | Precoding calculation |
| SINR Calculation | Reduction | 256 | Channel quality |
| MCS Selection | Lookup table | 128 | Modulation selection |

### 4.2 cuMAC Subcontext Management

```cpp
// File: cumacSubcontext.h
class cumac::cumacSubcontext {
private:
    // GPU Handlers
    cumac::mcSinrCalHndl_t mcSinrGpu;       // SINR calculation
    cumac::mcUeSelHndl_t mcUeSelGpu;        // UE selection
    cumac::mcSchdHndl_t mcSchGpu;           // Scheduling
    cumac::mcsSelLUTHndl_t mcMcsSelGpu;     // MCS selection
    cumac::mcLayerSelHndl_t mcLayerSelGpu;  // Layer selection

    // CPU Handlers (fallback)
    cumac::mcUeSelCpuHndl_t mcUeSelCpu;
    cumac::rrUeSelCpuHndl_t rrUeSelCpu;
    cumac::mcSchdCpuHndl_t mcSchCpu;
    cumac::rrSchdCpuHndl_t rrSchCpu;

public:
    void setup(const std::string& tvFilename, ...);

    void run(cudaStream_t strm) {
        // Launch kernels asynchronously
        mcSinrGpu.launch(strm);
        mcUeSelGpu.launch(strm);
        mcSchGpu.launch(strm);
        mcMcsSelGpu.launch(strm);
        mcLayerSelGpu.launch(strm);
        // Caller responsible for synchronization
    }
};
```

### 4.3 GPU Processing Pipeline

```
Stage 1: IQ Sample Reception
    ├─ Fronthaul decompression
    └─ Buffer management

Stage 2: Time-Domain Processing
    ├─ CP removal
    └─ Windowing

Stage 3: Frequency-Domain Processing
    ├─ FFT transformation
    ├─ Resource element demapping
    └─ Channel estimation

Stage 4: PRACH Detection (GPU-Accelerated)
    ├─ Reference sequence generation (Zadoff-Chu)
    ├─ Cross-correlation (multi-antenna)
    ├─ Peak detection
    ├─ Timing advance calculation
    └─ Power measurement

Stage 5: Multi-Cell Processing (GPU-Accelerated)
    ├─ UE selection
    ├─ PRB allocation
    ├─ Beamforming calculation
    ├─ Layer selection
    └─ MCS mapping

Timing: ~680μs typical (within 2ms budget)
```

---

## 5. Synchronization Mechanisms

### 5.1 Lock-Free Operations

#### Atomic Slot Command Updates
```cpp
// Lock-free circular buffer index update
void update_slot_cmds_indexes() {
    current_slot_cmd_index = (current_slot_cmd_index + 1) %
                             slot_command_array.size();
}

// Atomic tick updates with memory ordering
void tick_received(std::chrono::nanoseconds& timestamp) {
    current_tick_.store(timestamp, std::memory_order_release);
    ss_tick.store(new_tick, std::memory_order_release);
}
```

#### Benefits
- No spinlocks on critical path
- Reduced cache line contention
- Predictable latency
- Better CPU utilization

### 5.2 Event-Driven I/O (EPoll)

```cpp
// File: nv_phy_epoll_context.hpp
class phy_epoll_context {
private:
    int epoll_fd;
    std::unordered_map<int, event_callback*> fd_cache;
    std::vector<epoll_event> epoll_events;

public:
    void add_fd(int fd, event_callback* cb, uint32_t events = EPOLLIN) {
        epoll_event event;
        event.events = events;
        event.data.ptr = cb;
        epoll_ctl(epoll_fd, EPOLL_CTL_ADD, fd, &event);
        fd_cache[fd] = cb;
    }

    void wait_for_events(int timeout_ms = -1) {
        int n = epoll_wait(epoll_fd, epoll_events.data(),
                          epoll_events.size(), timeout_ms);
        for (int i = 0; i < n; ++i) {
            auto* cb = static_cast<event_callback*>(
                epoll_events[i].data.ptr
            );
            cb->handle_event(epoll_events[i].events);
        }
    }
};
```

### 5.3 Pre-Allocated Queue System

```cpp
template<typename T>
class nv_preallocated_queue {
private:
    std::vector<T> buffer;
    size_t head, tail;
    std::atomic<size_t> size;

public:
    nv_preallocated_queue(size_t capacity) :
        buffer(capacity), head(0), tail(0), size(0) {}

    bool push(const T& item) {
        if (size.load() >= buffer.size()) return false;
        buffer[tail] = item;
        tail = (tail + 1) % buffer.size();
        size.fetch_add(1);
        return true;
    }

    bool pop(T& item) {
        if (size.load() == 0) return false;
        item = buffer[head];
        head = (head + 1) % buffer.size();
        size.fetch_sub(1);
        return true;
    }
};
```

### 5.4 Slot Map Synchronization

```cpp
// Circular buffer for slot tracking
const uint32_t SLOT_MAP_SIZE = 512;

uint32_t slot_to_map(uint32_t sfn, uint32_t slot_in_frame) {
    // μ=1 (30 kHz SCS): 20 slots per frame
    uint32_t total_slots = sfn * 20 + slot_in_frame;
    return total_slots % SLOT_MAP_SIZE;
}

// Example: SFN 961.19 → Slot Map 448
// (961 * 20 + 19) % 512 = 19239 % 512 = 448
```

---

## 6. Control Plane vs Data Plane

### 6.1 Control Plane Components

#### cuPHY-CP Responsibilities
- Message parsing and routing
- Slot command generation
- Thread pool management
- Timing synchronization
- L2-L1 IPC communication

#### cuMAC-CP Responsibilities
- Scheduling task coordination
- Configuration management
- FAPI message formatting
- Resource allocation decisions

### 6.2 Data Plane Components

#### cuPHY Responsibilities
- Physical layer signal processing
- CUDA kernel execution
- FFT and correlation operations
- Channel estimation and equalization

#### cuMAC Responsibilities
- Multi-cell scheduling algorithms
- Beamforming calculations
- SINR and MCS selection
- PRB allocation

### 6.3 Message Flow and Protocols

#### Message Types
```cpp
// File: cumac_msg.h
typedef enum {
    // Configuration messages
    CUMAC_PARAM_REQUEST     = 0x00,
    CUMAC_PARAM_RESPONSE    = 0x01,
    CUMAC_CONFIG_REQUEST    = 0x02,
    CUMAC_CONFIG_RESPONSE   = 0x03,
    CUMAC_START_REQUEST     = 0x04,
    CUMAC_STOP_REQUEST      = 0x05,

    // TTI messages
    CUMAC_DL_TTI_REQUEST    = 0x80,
    CUMAC_UL_TTI_REQUEST    = 0x81,
    CUMAC_SCH_TTI_REQUEST   = 0x82,
    CUMAC_SCH_TTI_RESPONSE  = 0x83,
} cumac_msg_type_t;
```

#### Task Types
```cpp
typedef enum {
    CUMAC_TASK_UE_SELECTION    = 0,  // multiCellUeSelection
    CUMAC_TASK_PRB_ALLOCATION  = 1,  // multiCellScheduler
    CUMAC_TASK_LAYER_SELECTION = 2,  // multiCellLayerSel
    CUMAC_TASK_MCS_SELECTION   = 3,  // mcsSelectionLUT
} cumac_task_type_t;
```

#### Message Flow Diagram
```
MAC Layer
    ↓ [UL_TTI.request]
PHY-MAC Transport (IPC)
    ↓ [Shared Memory + Mutex]
PHY_module (EPoll receives)
    ↓ [Message dispatcher]
PHY Instance (Worker)
    ↓ [Process slot command]
GPU Kernels
    ↓ [Detection results]
Callback Handler
    ↓ [RACH.indication]
MAC Layer
```

---

## 7. Performance Optimization

### 7.1 CPU Affinity and NUMA

#### Core Assignment Strategy
```
NUMA Node 0 (Cores 0-5):
├─ Cores 0-3: Control plane, orchestration
├─ Core 4: UlPhyDriver04 (Uplink)
└─ Core 5: UlPhyDriver05 (Uplink)

NUMA Node 1 (Cores 6-11):
├─ Cores 6-7: UlPhyDriver06-07 (Uplink)
└─ Cores 8-11: DlPhyDriver08-11 (Downlink)

Benefits:
- Minimized cross-NUMA traffic
- Local memory access
- Better cache utilization
```

#### Thread Affinity Configuration
```cpp
void set_thread_affinity(int core_id) {
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(core_id, &cpuset);
    pthread_setaffinity_np(pthread_self(), sizeof(cpuset), &cpuset);
}
```

### 7.2 Memory Optimization

#### Pre-Allocation Strategy
```cpp
// Pre-allocate all buffers at initialization
class BufferPool {
    std::vector<Buffer> buffers;
    std::queue<Buffer*> available;
    std::mutex lock;

public:
    BufferPool(size_t count, size_t size) {
        buffers.reserve(count);
        for (size_t i = 0; i < count; ++i) {
            buffers.emplace_back(size);
            available.push(&buffers.back());
        }
    }

    Buffer* acquire() {
        std::lock_guard<std::mutex> guard(lock);
        if (available.empty()) return nullptr;
        Buffer* buf = available.front();
        available.pop();
        return buf;
    }

    void release(Buffer* buf) {
        std::lock_guard<std::mutex> guard(lock);
        available.push(buf);
    }
};
```

#### Cache-Friendly Data Structures
- Aligned allocations for SIMD operations
- Structure-of-Arrays (SoA) for GPU data
- Contiguous memory for sequential access

### 7.3 Parallel Processing Patterns

#### Pattern 1: Producer-Consumer
```
MAC (Producer) → Shared Queue → PHY_module (Consumer)
                 ↓
            EPoll notification
                 ↓
         Worker threads spawn
```

#### Pattern 2: Pipeline
```
OrderEntity → PhyPrachAggr → GPU → Callback
(Decompress)  (Orchestrate)  (Compute) (Results)
```

#### Pattern 3: Worker Pool
```
Task Queue: [Task1, Task2, Task3, ...]
    ↓
Worker Pool: [W1, W2, W3, W4]
    ↓
Parallel Execution
    ↓
Result Aggregation
```

### 7.4 Timeout and Recovery

#### Timeout Monitoring
```cpp
void monitor_timeout() {
    auto start = std::chrono::steady_clock::now();
    const auto timeout = std::chrono::nanoseconds(10000000);  // 10ms

    while (!all_channels_ready()) {
        auto elapsed = std::chrono::steady_clock::now() - start;
        if (elapsed > timeout) {
            LOG(ERROR) << "PRACH AGGR timeout after "
                      << elapsed.count() << "ns";
            trigger_recovery();
            return;
        }
        std::this_thread::sleep_for(std::chrono::microseconds(100));
    }
}
```

#### L1 Recovery Sequence
```
Timeout Detection
    ↓
Set in_recovery flag
    ↓
Drop incoming messages (4+ slots)
    ↓
Reset L1 state
    ↓
Log recovery status
    ↓
Resume normal operation
```

---

## 8. Configuration and Deployment

### 8.1 YAML Configuration

```yaml
# File: cuphycontroller_F08.yaml
phy_module:
  # Thread pool configuration
  num_threads: 8
  thread_affinity: true
  uplink_threads: 4
  downlink_threads: 4

  # Thread mapping
  thread_mapping:
    - thread_id: 4
      cpu_core: 4
      type: uplink
      name: "UlPhyDriver04"
    - thread_id: 5
      cpu_core: 5
      type: uplink
      name: "UlPhyDriver05"
    - thread_id: 8
      cpu_core: 8
      type: downlink
      name: "DlPhyDriver08"

  # PRACH configuration
  prach:
    max_occasions: 4
    timeout_ms: 10
    workspace_size_mb: 4
    detection_threshold: 0.5

  # Cell configuration
  cells:
    - cell_id: 0
      carrier_aggregation: true
      paired_with: 1
    - cell_id: 1
      carrier_aggregation: true
      paired_with: 0
```

### 8.2 Build Configuration

```cmake
# CMake configuration for optimal performance
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -march=native -mtune=native")
set(CMAKE_CUDA_FLAGS "-arch=sm_80 -O3")

# Enable parallel compilation
set(CMAKE_BUILD_PARALLEL_LEVEL 8)

# Link with performance libraries
target_link_libraries(cuphy
    pthread
    numa
    cudart
    cufft
    cublas
)
```

### 8.3 Runtime Parameters

```bash
# Launch script with optimal settings
#!/bin/bash

# Set CPU governor to performance
sudo cpupower frequency-set -g performance

# Disable CPU frequency scaling
echo 1 > /sys/devices/system/cpu/intel_pstate/no_turbo

# Set GPU to maximum performance
sudo nvidia-smi -pm 1
sudo nvidia-smi -pl 300  # Power limit in watts

# Launch application with real-time priority
sudo nice -n -20 ./aerial_cubb_app \
    --config config/cuphycontroller_F08.yaml \
    --threads 8 \
    --gpu-id 0
```

---

## 9. Key Implementation Files

### 9.1 Core Threading Files

| File | Purpose | Key Functions |
|------|---------|--------------|
| `nv_phy_module.hpp` | Main orchestrator | `thread_func()`, `dispatch()` |
| `nv_phy_instance.hpp` | Worker instances | `process_slot()`, `execute()` |
| `nv_phy_epoll_context.hpp` | Event I/O | `wait_for_events()`, `add_fd()` |
| `nv_phy_driver_proxy.hpp` | Driver singleton | `get_instance()`, `configure()` |
| `nv_tick_generator.cpp` | Timing sync | `generate_tick()`, `sync()` |

### 9.2 GPU Processing Files

| File | Purpose | Kernels |
|------|---------|---------|
| `multiCellScheduler.cu` | Scheduling | `multiCellSchedulerKernel_svdMmseIrc()` |
| `multiCellBeamform.cu` | Beamforming | `beamformingKernel()` |
| `multiCellSinrCal.cu` | SINR calc | `sinrCalculationKernel()` |
| `cumacSubcontext.h` | GPU management | `setup()`, `run()` |

### 9.3 Message Protocol Files

| File | Purpose | Structures |
|------|---------|-----------|
| `cumac_msg.h` | MAC messages | `cumac_msg_t`, task types |
| `scf_5g_fapi.h` | FAPI protocol | FAPI message formats |
| `scf_5g_slot_commands_common.hpp` | Slot commands | Command structures |

---

## 10. Architecture Diagrams

### 10.1 Complete System Architecture

```
┌─────────────────────────────────────────────────────────────────────────┐
│                           MAC Layer (L2 Stack)                           │
│                    [Scheduling, RLC, PDCP, RRC]                          │
├─────────────────────────────────────────────────────────────────────────┤
│                      PHY-MAC Transport Layer (IPC)                       │
│              [Shared Memory | Mutex Protection | EPoll]                  │
├─────────────────────────────────────────────────────────────────────────┤
│                       PHY Control Plane (cuPHY-CP)                       │
│ ┌───────────────────────────────────────────────────────────────────┐   │
│ │                        PHY_module (Main Thread)                    │   │
│ │  ┌──────────────────────────────────────────────────────────┐    │   │
│ │  │ • EPoll Context (Event-driven I/O)                       │    │   │
│ │  │ • Module Dispatcher (Message routing)                    │    │   │
│ │  │ • Tick Generator (Timing synchronization)                │    │   │
│ │  │ • Slot Command Array (512-entry circular buffer)         │    │   │
│ │  └──────────────────────────────────────────────────────────┘    │   │
│ └───────────────────────────────────────────────────────────────────┘   │
├─────────────────────────────────────────────────────────────────────────┤
│                         Worker Thread Pool                               │
│ ┌────────────────┐ ┌────────────────┐ ┌────────────────┐ ┌────────────┐│
│ │ UlPhyDriver04  │ │ UlPhyDriver05  │ │ UlPhyDriver06  │ │DlPhyDriver08││
│ │   Core 4       │ │   Core 5       │ │   Core 6       │ │   Core 8    ││
│ │ PRACH/PUSCH    │ │ PRACH/PUSCH    │ │ PRACH/PUSCH    │ │   PDSCH     ││
│ │ AGGR Tasks     │ │ AGGR Tasks     │ │ AGGR Tasks     │ │ Processing  ││
│ │ Cells [0,1]    │ │ Cells [0,1]    │ │ Cells [0,1]    │ │ Cells [0,1] ││
│ └────────────────┘ └────────────────┘ └────────────────┘ └────────────┘│
├─────────────────────────────────────────────────────────────────────────┤
│                      Task Management Layer                               │
│ ┌───────────────────────────────────────────────────────────────────┐   │
│ │ • PhyPrachAggr Tasks (Per-thread counters: AGGR 0, 1, 2, 3...)   │   │
│ │ • Channel Synchronization (4 channels per task)                  │   │
│ │ • Timeout Monitoring (10ms PRACH_AGGR_TIMEOUT)                   │   │
│ │ • State Machine (CREATED→SETUP→WAITING→RUNNING→CALLBACK→CLEANUP) │   │
│ └───────────────────────────────────────────────────────────────────┘   │
├─────────────────────────────────────────────────────────────────────────┤
│                    GPU Processing Layer (CUDA)                           │
│ ┌───────────────────────────────────────────────────────────────────┐   │
│ │                          cuPHY Kernels                             │   │
│ │ • PRACH Detection (FFT, correlation, peak detection)              │   │
│ │ • PUSCH Decoding (equalization, demodulation, turbo decoding)     │   │
│ │ • PUCCH Processing (format detection, UCI extraction)             │   │
│ │ • Channel Estimation (pilot extraction, interpolation)            │   │
│ └───────────────────────────────────────────────────────────────────┘   │
│ ┌───────────────────────────────────────────────────────────────────┐   │
│ │                          cuMAC Kernels                             │   │
│ │ • Multi-Cell Scheduling (UE selection, PRB allocation)            │   │
│ │ • Beamforming (SVD, MMSE, IRC precoding)                          │   │
│ │ • SINR Calculation (interference estimation)                      │   │
│ │ • MCS Selection (lookup table, link adaptation)                   │   │
│ │ • Layer Selection (rank adaptation)                               │   │
│ └───────────────────────────────────────────────────────────────────┘   │
├─────────────────────────────────────────────────────────────────────────┤
│                         O-RAN Fronthaul                                  │
│                    [IQ Samples | Compression | Timing]                   │
└─────────────────────────────────────────────────────────────────────────┘
```

### 10.2 Slot Processing Timeline

```
Time →
┌──────────────────────────────────────────────────────────────────────────┐
│ Slot N-1 │         Slot N          │        Slot N+1         │ Slot N+2  │
└──────────────────────────────────────────────────────────────────────────┘
           ↑
           MAC sends UL_TTI.request
           │
           ├─[0μs]────── EPoll receives message
           │
           ├─[10μs]───── PHY_module dispatches to worker
           │
           ├─[20μs]───── Worker creates AGGR task
           │
           ├─[50μs]───── Channels ready, GPU launch
           │
           ├─[100μs]──── GPU kernel execution starts
           │             ├─ FFT processing
           │             ├─ Correlation computation
           │             ├─ Peak detection
           │             └─ Timing advance calculation
           │
           ├─[780μs]──── GPU kernel completes
           │
           ├─[800μs]──── Callback processes results
           │
           ├─[850μs]──── RACH.indication built
           │
           └─[900μs]──── Response sent to MAC

           Total: ~900μs (well within 2ms budget)
```

### 10.3 Memory Architecture

```
┌─────────────────────────────────────────────────────────────────────────┐
│                           System Memory                                  │
├─────────────────────────────────────────────────────────────────────────┤
│                         NUMA Node 0                                      │
│ ┌───────────────────────────────────────────────────────────────────┐   │
│ │ Control Plane Memory                                              │   │
│ │ • PHY_module data structures                                      │   │
│ │ • Message queues (pre-allocated)                                  │   │
│ │ • Slot command array                                              │   │
│ └───────────────────────────────────────────────────────────────────┘   │
│ ┌───────────────────────────────────────────────────────────────────┐   │
│ │ Worker Thread Memory (Cores 4-5)                                  │   │
│ │ • Thread-local storage                                            │   │
│ │ • Task state machines                                             │   │
│ │ • IQ sample buffers                                               │   │
│ └───────────────────────────────────────────────────────────────────┘   │
├─────────────────────────────────────────────────────────────────────────┤
│                         NUMA Node 1                                      │
│ ┌───────────────────────────────────────────────────────────────────┐   │
│ │ Worker Thread Memory (Cores 6-11)                                 │   │
│ │ • Additional uplink buffers                                       │   │
│ │ • Downlink processing buffers                                     │   │
│ │ • Result aggregation buffers                                      │   │
│ └───────────────────────────────────────────────────────────────────┘   │
├─────────────────────────────────────────────────────────────────────────┤
│                         GPU Memory (HBM)                                 │
│ ┌───────────────────────────────────────────────────────────────────┐   │
│ │ • CUDA kernel workspace                                           │   │
│ │ • FFT twiddle factors                                             │   │
│ │ • Reference sequences (Zadoff-Chu)                                │   │
│ │ • Correlation buffers                                             │   │
│ │ • Detection results                                               │   │
│ └───────────────────────────────────────────────────────────────────┘   │
├─────────────────────────────────────────────────────────────────────────┤
│                      Shared Memory (IPC)                                 │
│ ┌───────────────────────────────────────────────────────────────────┐   │
│ │ • MAC-PHY message buffers                                         │   │
│ │ • Transport blocks                                                 │   │
│ │ • Control information                                             │   │
│ └───────────────────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────────────────┘
```

---

## Summary

The NVIDIA Aerial cuBB architecture represents a state-of-the-art implementation of 5G/6G baseband processing, combining:

1. **Multi-tier parallelism** across CPU threads, tasks, cells, and GPU kernels
2. **Lock-free synchronization** for minimal contention and predictable latency
3. **Event-driven I/O** using epoll for efficient message handling
4. **Dynamic load balancing** through round-robin slot distribution
5. **GPU acceleration** for compute-intensive signal processing
6. **NUMA-aware** thread and memory placement
7. **Pre-allocated resources** to eliminate runtime allocation overhead
8. **Comprehensive timeout and recovery** mechanisms for resilience

This architecture achieves sub-millisecond processing latency while maintaining the flexibility to scale across multiple cells, carriers, and processing cores, making it suitable for both current 5G deployments and future 6G requirements.