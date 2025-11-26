# CuPHYController: Roles and Responsibilities

## Executive Summary

The **cuPHYController** is the critical initialization and configuration management component of the NVIDIA Aerial cuBB stack. It acts as the bridge between high-level system configuration (YAML files) and low-level PHY driver operations, managing the entire lifecycle of the PHY layer from initialization through runtime operation to graceful shutdown.

## Table of Contents

1. [Component Overview](#1-component-overview)
2. [Primary Roles](#2-primary-roles)
3. [Directory Structure](#3-directory-structure)
4. [Core Components](#4-core-components)
5. [Configuration Management](#5-configuration-management)
6. [Resource Management](#6-resource-management)
7. [State Management](#7-state-management)
8. [Communication Interfaces](#8-communication-interfaces)
9. [Advanced Features](#9-advanced-features)
10. [Error Handling](#10-error-handling)
11. [Key API Functions](#11-key-api-functions)
12. [Configuration Files](#12-configuration-files)

---

## 1. Component Overview

### Location
```
/cuPHY-CP/cuphycontroller/
```

### Primary Purpose
The cuPHYController serves as the **configuration and initialization framework** that:
- Parses and validates system configurations
- Initializes and manages the PHY driver lifecycle
- Orchestrates resource allocation (GPUs, threads, memory)
- Manages cell creation and activation
- Provides L2 adapter integration
- Handles runtime configuration updates

### Key Characteristics
- **Configuration-driven**: 100+ configurable parameters
- **Platform-agnostic**: Supports multiple hardware configurations
- **Mode-flexible**: Normal, validation, and standalone modes
- **Resource-aware**: Intelligent GPU/CPU/memory allocation
- **Error-resilient**: Comprehensive validation and fallback logic

---

## 2. Primary Roles

### 2.1 Configuration Parser and Validator
```cpp
// Parse complex YAML configurations
YamlParser parser;
parser.parse_file(config_file);
parser.parse_cuphydriver_configs();
parser.parse_cell_configs();
```

**Responsibilities**:
- Parse main cuPHYController YAML files
- Parse L2 adapter configurations
- Parse test vector configurations
- Validate all parameters before initialization
- Detect configuration conflicts and duplicates
- Provide default values for optional parameters

### 2.2 PHY Driver Lifecycle Manager
```cpp
// Initialize PHY driver
pc_init_phydriver(config, worker_descs, phydriver_handle);

// Cell lifecycle management
l1_cell_create(cell_config, cell_handle);
l1_cell_start(cell_handle);
l1_cell_stop(cell_handle);
l1_cell_destroy(cell_handle);

// Finalize PHY driver
pc_finalize_phydriver(phydriver_handle);
```

**Responsibilities**:
- Initialize cuPHY driver context
- Create and manage cell instances
- Start/stop cell processing
- Clean shutdown and resource release

### 2.3 Resource Orchestrator
```yaml
# Resource allocation configuration
workers_ul: [5, 6, 7]        # UL processing cores
workers_dl: [11, 12, 13, 14] # DL processing cores
gpus: [0, 1]                  # GPU devices
workers_sched_priority: 95    # Real-time priority
```

**Responsibilities**:
- Allocate GPU resources and CUDA contexts
- Bind worker threads to CPU cores
- Configure memory pools and buffers
- Set scheduling priorities
- Manage NIC assignments for fronthaul

### 2.4 Work Scheduler and Dispatcher
```cpp
// Enqueue PHY work for slot processing
l1_enqueue_phy_work(
    phydriver_handle,
    slot_command,
    slot_number,
    cell_id
);
```

**Responsibilities**:
- Submit slot commands to PHY workers
- Manage work distribution across threads
- Handle callback registration for results
- Coordinate timing synchronization

### 2.5 L2 Adapter Interface Provider
```cpp
// L2 adapter worker routine
void* pc_l2_adapter(void* hdl) {
    // Receive L2 commands
    // Translate to PHY work
    // Enqueue for processing
    // Return results via callbacks
}
```

**Responsibilities**:
- Bridge between L2 (MAC) and L1 (PHY)
- Handle slot scheduling commands
- Manage timing advancement
- Provide result callbacks to MAC

---

## 3. Directory Structure

```
cuPHY-CP/cuphycontroller/
│
├── include/                    # Header files
│   ├── cuphydriver.hpp        # PHY driver interface definitions
│   └── yamlparser.hpp         # YAML parser class definitions
│
├── src/                       # Implementation files
│   ├── cuphydriver.cpp        # PHY driver implementation
│   └── yamlparser.cpp         # YAML parsing logic
│
├── examples/                  # Example applications
│   ├── cuphycontroller_scf.cpp # SCF L2 adapter example
│   └── CMakeLists.txt
│
├── config/                    # Configuration files (30+ variants)
│   ├── cuphycontroller_*.yaml # PHY controller configs
│   ├── l2_adapter_config_*.yaml # L2 adapter configs
│   ├── launch_pattern.yaml   # Slot scheduling patterns
│   └── nvipc_multi_instances.yaml # Multi-instance configs
│
├── lib/                       # Library build outputs
└── CMakeLists.txt            # Build configuration
```

---

## 4. Core Components

### 4.1 CuPHY Driver (cuphydriver.hpp/cpp)

#### Opaque Handle Types
```cpp
typedef void* phydriver_handle;     // PHY driver context
typedef void* phydriverwrk_handle;  // Worker thread handle
```

#### Key Classes and Functions
| Component | Purpose |
|-----------|---------|
| `pc_init_phydriver()` | Initialize PHY driver with configuration |
| `pc_finalize_phydriver()` | Clean shutdown of PHY driver |
| `l1_cell_*()` functions | Cell lifecycle management |
| `l1_worker_*()` functions | Worker thread management |
| `l1_enqueue_phy_work()` | Submit work for processing |
| `l1_set_output_callback()` | Register result callbacks |

### 4.2 YAML Parser (yamlparser.hpp/cpp)

#### Parser Class Hierarchy
```cpp
class YamlParser {
    cuphydriver_config config;     // Main configuration
    vector<cell_phy_info> cells;   // Cell-specific configs
    vector<cell_mplane_info> mplane; // Fronthaul configs

    void parse_file(string filename);
    void parse_cuphydriver_configs();
    void parse_cell_configs();
    void validate_all();
};
```

#### Configuration Structures
```cpp
struct cuphydriver_config {
    // 100+ parameters including:
    int validation;           // 0=disabled, 1=enabled
    int standalone;          // 0=normal, 1=standalone
    vector<int> workers_ul;  // UL worker CPU cores
    vector<int> workers_dl;  // DL worker CPU cores
    vector<int> gpus;        // GPU device IDs
    int workers_sched_priority; // RT priority (95)
    // ... many more parameters
};

struct cell_phy_info {
    int cell_id;            // Unique cell identifier
    string cell_name;       // Descriptive name
    int mu;                 // Numerology (0-4)
    int nrbdl;              // DL bandwidth in RBs
    int nrbul;              // UL bandwidth in RBs
    // ... PHY parameters
};

struct cell_mplane_info {
    string mac_src;         // Source MAC address
    string mac_dst;         // Destination MAC address
    int vlan_id;            // VLAN tag
    int pcp;                // Priority code point
    // ... fronthaul parameters
};
```

---

## 5. Configuration Management

### 5.1 Configuration Hierarchy

```
Main Configuration (cuphydriver_config)
├── System Resources
│   ├── CPU cores (UL/DL workers)
│   ├── GPU devices
│   ├── NIC assignments
│   └── Memory allocations
├── Algorithm Selection
│   ├── PUSCH algorithms (chest, eq_coeff)
│   ├── SRS algorithms (chest, normalization)
│   ├── PDSCH algorithms (precoding)
│   └── PDCCH algorithms (detection)
├── Timing Parameters
│   ├── Slot timing offsets
│   ├── TTI advancement
│   ├── GPU timeouts
│   └── Fronthaul windows (T1a, Ta4, Tcp)
├── Feature Toggles
│   ├── GPU graphs (enable_ul/dl_cuphy_graphs)
│   ├── HARQ early detection
│   ├── Compression modes
│   └── Beamforming (static/dynamic)
└── Debug/Monitoring
    ├── Log levels
    ├── Profiling duration
    ├── Tracing options
    └── PCAP logging
```

### 5.2 Validation Rules

The parser enforces strict validation:

```cpp
// MAC address format validation
bool validate_mac(string mac) {
    // Must match XX:XX:XX:XX:XX:XX format
}

// PCI device existence check
bool validate_pci_device(string addr) {
    // Check /sys/bus/pci/devices/[addr]
}

// Port availability check
bool validate_port(int port) {
    // Ensure no conflicts with existing services
}

// Cell ID uniqueness
bool validate_cell_ids(vector<int> ids) {
    // No duplicates allowed
}

// Parameter range validation
bool validate_range(int value, int min, int max) {
    // Ensure value within acceptable range
}
```

### 5.3 Configuration Loading Process

```
1. Load YAML file
    ↓
2. Parse main configuration block
    ↓
3. Parse cell configurations (iterate)
    ↓
4. Validate all parameters
    ↓
5. Apply defaults for optional params
    ↓
6. Check device existence (PCI, NIC)
    ↓
7. Verify no conflicts (ports, IDs)
    ↓
8. Store in structured format
    ↓
9. Pass to PHY driver initialization
```

---

## 6. Resource Management

### 6.1 GPU Resource Allocation

#### GPU Selection and Assignment
```yaml
gpus: [0, 1]              # Use GPU 0 and 1
cell_groups: 2            # Group cells across GPUs
cell_group_count: [1, 1]  # 1 cell per GPU
```

#### Streaming Multiprocessor (SM) Allocation
```yaml
# Fine-grained SM allocation per channel
mps_sm_pusch: 100   # PUSCH processing
mps_sm_pucch: 2     # PUCCH processing
mps_sm_prach: 2     # PRACH detection
mps_sm_srs: 16      # SRS processing
mps_sm_pdsch: 102   # PDSCH processing
mps_sm_pdcch: 10    # PDCCH processing
mps_sm_gpu_comms: 16 # GPU communication
```

### 6.2 CPU Thread Management

#### Worker Thread Configuration
```yaml
# Worker assignment to CPU cores
workers_ul: [5, 6, 7]           # 3 UL workers
workers_dl: [11, 12, 13, 14]    # 4 DL workers
workers_dl_validation: [15]     # Validation worker
workers_sched_priority: 95      # Real-time priority

# Additional threads
dpdk_thread: 3                  # DPDK fronthaul
h2d_thread: 9                   # Host-to-Device copy
```

#### Thread Creation and Binding
```cpp
// Create worker with specific CPU affinity
l1_worker_start_generic(
    phydriver_handle,
    worker_routine,
    cpu_core,
    priority,
    worker_handle
);
```

### 6.3 Memory Management

#### Buffer Pool Configuration
```yaml
# HARQ buffer management
harq_context_size: 16           # Context pool size
harq_buffer_num_per_ue: 8       # Buffers per UE
enable_harq_offload: 1          # GPU offload

# SRS buffer allocation
srs_chest_buff_num: 100         # Chest buffer count

# UL input buffers
ul_in_buff_per_cell: 5          # Per-cell buffers
```

---

## 7. State Management

### 7.1 PHY Driver State Machine

```
┌──────────────────┐
│  UNINITIALIZED   │
└────────┬─────────┘
         │ pc_init_phydriver()
         ↓
┌──────────────────┐
│   INITIALIZED    │
└────────┬─────────┘
         │ l1_cell_create()
         ↓
┌──────────────────┐
│  CELL_CREATED    │
└────────┬─────────┘
         │ l1_cell_start()
         ↓
┌──────────────────┐
│   CELL_ACTIVE    │←───┐
└────────┬─────────┘    │
         │              │ l1_enqueue_phy_work()
         ↓              │
┌──────────────────┐    │
│   PROCESSING     │────┘
└────────┬─────────┘
         │ l1_cell_stop()
         ↓
┌──────────────────┐
│   CELL_STOPPED   │
└────────┬─────────┘
         │ l1_cell_destroy()
         ↓
┌──────────────────┐
│  CELL_DESTROYED  │
└────────┬─────────┘
         │ pc_finalize_phydriver()
         ↓
┌──────────────────┐
│    FINALIZED     │
└──────────────────┘
```

### 7.2 Worker Thread States

```cpp
enum WorkerState {
    WORKER_CREATED,      // Thread created but not started
    WORKER_RUNNING,      // Active processing
    WORKER_IDLE,         // Waiting for work
    WORKER_STOPPING,     // Shutdown requested
    WORKER_STOPPED       // Thread terminated
};
```

### 7.3 Cell State Transitions

```cpp
// Cell activation sequence
l1_cell_create(config, &cell_handle);
l1_cell_config_request(cell_handle, params);
l1_cell_start(cell_handle);  // → ACTIVE state

// Runtime updates (cell remains ACTIVE)
l1_cell_update_cell_config(cell_handle, new_params);

// Cell deactivation sequence
l1_cell_stop(cell_handle);   // → STOPPED state
l1_cell_destroy(cell_handle);
```

---

## 8. Communication Interfaces

### 8.1 L1 API Interface

The controller uses the L1 API for PHY operations:

```cpp
// L1 API usage pattern
class L1Interface {
    // Context management
    l1_init(config);
    l1_finalize();

    // Cell operations
    l1_cell_create();
    l1_cell_start();
    l1_cell_stop();
    l1_cell_destroy();

    // Work submission
    l1_enqueue_phy_work(slot_command);

    // Result callbacks
    l1_set_output_callback(callback_func);
};
```

### 8.2 L2 Adapter Interface

Bridge between MAC and PHY layers:

```cpp
// L2 Adapter message flow
L2 (MAC) → L2_Adapter → cuPHYController → PHY_Driver
    ↓                                          ↓
[Slot Commands]                        [PHY Processing]
    ↓                                          ↓
[TTI Messages]                         [GPU Execution]
    ↓                                          ↓
L2 (MAC) ← L2_Adapter ← cuPHYController ← [Results]
```

### 8.3 Fronthaul Interface

M-plane configuration for O-RAN fronthaul:

```yaml
# Fronthaul configuration
nic_interface: 0000:17:00.0    # PCI address
mac_src: "00:11:22:33:44:55"   # Source MAC
mac_dst: "00:66:77:88:99:AA"   # Destination MAC
vlan_id: 100                   # VLAN tag
pcp: 7                          # Priority

# Timing windows
t1a_min: 345000                # T1a minimum (ns)
t1a_max: 545000                # T1a maximum (ns)
ta4_min: 0                      # Ta4 minimum (ns)
ta4_max: 350000                # Ta4 maximum (ns)
tcp: 299000                     # Tcp advance (ns)
```

### 8.4 OAM Interface

Operations, Administration, and Maintenance:

```cpp
// OAM integration
CuphyOAM* oam = CuphyOAM::getInstance();
oam->init_everything();

// OAM services
oam->register_cell(cell_id);
oam->update_metrics();
oam->handle_alarms();
oam->process_configuration_updates();
```

---

## 9. Advanced Features

### 9.1 Beamforming Support

#### Static Beamforming
```yaml
bfw_static_beam_id_min: 1
bfw_static_beam_id_max: 16527
bfw_window_size: 1000
```

#### Dynamic Beamforming
```yaml
bfw_dynamic_beam_id_min: 16528
bfw_dynamic_beam_id_max: 65535
bfw_buffering_time_dl_data: 500  # microseconds
```

### 9.2 Standalone Testing Mode

```yaml
standalone: 1                    # Enable standalone mode
test_mac_mode: 2                # Test MAC type
# Test vector files
pusch_testvect: "test_data/pusch_test.h5"
pucch_testvect: "test_data/pucch_test.h5"
prach_testvect: "test_data/prach_test.h5"
```

### 9.3 GPU Graph Optimization

```yaml
enable_ul_cuphy_graphs: 1       # UL GPU graphs
enable_dl_cuphy_graphs: 1       # DL GPU graphs
# Improves kernel launch overhead
```

### 9.4 Compression Modes

```yaml
compression_method_ul: 1         # BFP compression
compression_method_dl: 1         # BFP compression
compression_bits_ul: 9          # 9-bit compression
compression_bits_dl: 9          # 9-bit compression
```

### 9.5 Multi-Instance Support

```yaml
# Multiple L2 adapter instances
backend_address: "127.0.0.1:8000,127.0.0.1:8001"
backend_threads: 2
multi_recv: 1
```

---

## 10. Error Handling

### 10.1 Configuration Validation Errors

```cpp
// Detailed error messages
if (!validate_mac_format(mac)) {
    LOG(ERROR) << "Invalid MAC format: " << mac
               << ". Expected format: XX:XX:XX:XX:XX:XX";
    return -1;
}

if (!check_pci_device_exists(pci_addr)) {
    LOG(ERROR) << "PCI device not found: " << pci_addr
               << ". Check 'lspci' output and device binding";
    return -1;
}
```

### 10.2 Runtime Error Recovery

```cpp
// Graceful degradation
try {
    l1_enqueue_phy_work(work);
} catch (const std::exception& e) {
    LOG(WARNING) << "Work enqueue failed: " << e.what();
    // Try recovery or skip slot
    if (can_recover()) {
        reset_worker();
        retry_enqueue(work);
    }
}
```

### 10.3 Signal Handling

```cpp
// Clean shutdown on signals
signal(SIGINT, signal_handler);
signal(SIGTERM, signal_handler);

void signal_handler(int sig) {
    LOG(INFO) << "Received signal " << sig << ", shutting down";
    // Graceful shutdown sequence
    stop_all_cells();
    finalize_phy_driver();
    exit(0);
}
```

---

## 11. Key API Functions

### 11.1 Initialization Functions

| Function | Purpose | Parameters |
|----------|---------|------------|
| `pc_init_phydriver()` | Initialize PHY driver | config, worker_descs, handle |
| `l1_cell_create()` | Create cell instance | cell_config, cell_handle |
| `l1_cell_start()` | Activate cell | cell_handle |
| `l1_worker_start_generic()` | Start worker thread | routine, core, priority |

### 11.2 Runtime Functions

| Function | Purpose | Parameters |
|----------|---------|------------|
| `l1_enqueue_phy_work()` | Submit work | slot_cmd, slot_num, cell_id |
| `l1_cell_update_cell_config()` | Update config | cell_handle, new_params |
| `l1_set_output_callback()` | Register callback | callback_func, context |
| `l1_worker_check_exit()` | Check worker state | worker_handle |

### 11.3 Cleanup Functions

| Function | Purpose | Parameters |
|----------|---------|------------|
| `l1_cell_stop()` | Deactivate cell | cell_handle |
| `l1_cell_destroy()` | Destroy cell | cell_handle |
| `l1_worker_stop()` | Stop worker | worker_handle |
| `pc_finalize_phydriver()` | Cleanup driver | phydriver_handle |

---

## 12. Configuration Files

### 12.1 Main Configuration Files

| File Pattern | Purpose | Key Features |
|--------------|---------|--------------|
| `cuphycontroller_F08.yaml` | Single-cell F08 | Basic single-cell config |
| `cuphycontroller_F08_R750.yaml` | Multi-port F08 | R750 hardware support |
| `cuphycontroller_40C.yaml` | 4-cell Coolidge | Multi-cell deployment |
| `cuphycontroller_P5G_*.yaml` | Private 5G | Enterprise configurations |
| `cuphycontroller_SE_*.yaml` | Standalone Edge | Edge deployment configs |
| `cuphycontroller_nrSim_*.yaml` | Simulation | Testing configurations |

### 12.2 L2 Adapter Configurations

| File | Purpose |
|------|---------|
| `l2_adapter_config_F08.yaml` | F08 L2 adapter settings |
| `l2_adapter_config_interop.yaml` | Interop testing |
| `l2_adapter_config_R750.yaml` | R750 L2 adapter |

### 12.3 Specialized Configurations

| File | Purpose |
|------|---------|
| `launch_pattern.yaml` | Slot scheduling patterns |
| `nvipc_multi_instances.yaml` | Multi-instance IPC |
| `cuphyoam_config.yaml` | OAM service configuration |

---

## Summary

The cuPHYController serves as the **critical configuration and management layer** of the NVIDIA Aerial cuBB stack, with five primary roles:

1. **Configuration Management**: Parses and validates 100+ parameters from YAML files
2. **Lifecycle Management**: Controls PHY driver and cell initialization, operation, and shutdown
3. **Resource Orchestration**: Intelligently allocates GPUs, CPU cores, and memory
4. **Work Scheduling**: Distributes slot-based processing across worker threads
5. **Interface Bridge**: Connects L2 (MAC) with L1 (PHY) through adapters and callbacks

Its sophisticated design enables flexible deployment across various hardware platforms while maintaining strict real-time processing requirements for 5G/6G networks.