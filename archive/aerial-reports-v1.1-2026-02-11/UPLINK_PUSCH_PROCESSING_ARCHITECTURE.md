# NVIDIA Aerial cuBB - PUSCH Uplink Processing Architecture (v1.1)

> **Document Version:** v1.1
> **Last Updated:** 2026-02-11
> **Repository Context:** `cuBB-25-2-Codex` (NVIDIA Aerial CUDA-Accelerated RAN workspace)

## Report Context (Self-Contained)
- This is a standalone technical report for NVIDIA Aerial codepaths in this repository.
- Module names, APIs, and thread/task flows are interpreted from current workspace sources.
- Runtime behavior can vary by deployment YAML, build flags, GPU/NIC topology, and testbench mode.

## Reader Guide
- `Purpose`: what this report/component is intended to explain.
- `Flow`: end-to-end processing order and critical handoffs.
- `Configuration`: major knobs/defaults that affect behavior or performance.
- `Risks`: implementation caveats, operational pitfalls, and validation focus.

## Core Glossary
- `L1`: PHY runtime execution layer (cuPHY and cuPHY-CP driver paths).
- `L2`: Scheduler/control side producing PHY work (FAPI-like command domain).
- `FH`: Fronthaul transport path (O-RAN split data/control planes).
- `SFN/Slot`: 3GPP frame/slot timing identifiers.
- `UE`: User equipment context handled by channel pipelines.

## v1.1 Revision Notes
- Reformatted as a self-contained report with explicit context and reading map.
- Standardized terminology to reduce ambiguity across Aerial modules.
- Expanded report framing to support independent review without external notes.

## Table of Contents
1. [Executive Summary](#executive-summary)
2. [Architecture Overview](#architecture-overview)
3. [Multi-Threading Architecture](#multi-threading-architecture)
4. [Multi-Task Processing Architecture](#multi-task-processing-architecture)
5. [PUSCH Processing Pipeline](#pusch-processing-pipeline)
6. [Key Data Structures](#key-data-structures)
7. [Detailed Component Analysis](#detailed-component-analysis)
8. [Processing Flow Diagrams](#processing-flow-diagrams)
9. [Thread Synchronization](#thread-synchronization)
10. [Performance Considerations](#performance-considerations)

---

## Executive Summary

The NVIDIA Aerial cuBB PUSCH (Physical Uplink Shared Channel) processing system implements a sophisticated multi-threaded, multi-task architecture for handling 5G NR uplink data channels. The system leverages GPU acceleration through cuPHY while maintaining a robust CPU-based control plane architecture.

**Key Characteristics:**
- **Multi-threaded design**: Main message processing thread + worker thread pool
- **Slot-based processing**: 512-slot circular buffer for command orchestration
- **GPU-accelerated**: Asynchronous PUSCH decoding via NVIDIA cuPHY kernels
- **FAPI-compliant**: Implements SCF 5G FAPI interface for MAC-PHY communication
- **High throughput**: Supports up to 20 cells per slot with advanced load balancing

---

## Architecture Overview

### System Layers

```
┌─────────────────────────────────────────────────────────────────┐
│                        MAC Layer (L2)                            │
│                     (cuMAC or External MAC)                      │
└──────────────────────────┬──────────────────────────────────────┘
                           │ FAPI Messages (UL_TTI.request)
                           │ IPC (Shared Memory)
┌──────────────────────────▼──────────────────────────────────────┐
│                   L2 Adapter (cuPHY-CP)                          │
│  ┌────────────────────────────────────────────────────────────┐ │
│  │           PHY_module (Main Orchestrator)                   │ │
│  │  - msg_processing thread (FAPI message receiver)           │ │
│  │  - Slot command circular buffer (512 entries)              │ │
│  │  - Tick synchronization (atomic operations)                │ │
│  └────────────────────────┬───────────────────────────────────┘ │
│                           │                                      │
│  ┌────────────────────────▼───────────────────────────────────┐ │
│  │     scf_5g_fapi::phy instances (Per-Cell Workers)          │ │
│  │  - PUSCH PDU validation & parsing                          │ │
│  │  - Slot command preparation                                │ │
│  │  - Result callback generation                              │ │
│  └────────────────────────┬───────────────────────────────────┘ │
└───────────────────────────┼──────────────────────────────────────┘
                            │ Slot Commands
┌───────────────────────────▼──────────────────────────────────────┐
│              PHY Driver (cuPHY Interface)                         │
│  ┌──────────────────────────────────────────────────────────┐   │
│  │        UlPhyDriver Threads (Worker Pool)                 │   │
│  │  - UlPhyDriver04, UlPhyDriver05, ...                     │   │
│  │  - Round-robin slot distribution                         │   │
│  │  - GPU stream management                                 │   │
│  └──────────────────────┬───────────────────────────────────┘   │
└─────────────────────────┼────────────────────────────────────────┘
                          │ cuPHY API calls
┌─────────────────────────▼────────────────────────────────────────┐
│                    NVIDIA cuPHY (GPU Layer)                       │
│  - PUSCH channel estimation & equalization                       │
│  - LDPC decoding                                                 │
│  - CRC checking                                                  │
│  - UCI extraction (HARQ-ACK, CSI)                                │
└──────────────────────────┬───────────────────────────────────────┘
                           │ Callbacks
┌──────────────────────────▼───────────────────────────────────────┐
│              Result Indication to MAC                             │
│  - CRC.indication        - RX_DATA.indication                    │
│  - UCI.indication        - RX_PE_NOISE_VAR.indication            │
└───────────────────────────────────────────────────────────────────┘
```

---

## Multi-Threading Architecture

### Thread Hierarchy

The PUSCH processing system employs a hierarchical multi-threading model:

#### 1. **Main Message Processing Thread** (`msg_processing`)

**File**: `nv_phy_module.cpp:856`

**Responsibilities**:
- Receives FAPI messages from MAC via IPC (shared memory)
- Uses `epoll` for efficient event-driven message reception
- Dispatches messages to appropriate PHY instance workers
- Orchestrates slot command execution
- Manages slot synchronization across cells

**Thread Configuration**:
```cpp
// cuPHY-CP/cuphyl2adapter/lib/nvPHY/nv_phy_module.cpp:495-536
std::thread t(&PHY_module::thread_func, this);
pthread_setname_np(thread_.native_handle(), "msg_processing");

// CPU affinity and scheduling priority configurable via YAML:
// - thread_cfg_->cpu_affinity
// - thread_cfg_->sched_priority
```

**Key Operations**:
```cpp
void PHY_module::msg_processing() {
    while (transport_wrapper().rx_recv(smsg) >= 0) {
        // Dispatch to PHY instance
        if (phy_refs_[smsg.cell_id].get().on_msg(smsg)) {
            transport_wrapper().rx_release(smsg);
        }

        // Check if all cells ready for slot processing
        if (active_cell_bitmap &&
            (fapi_eom_rcvd_bitmap & active_cell_bitmap) == active_cell_bitmap) {
            process_phy_commands(slot_end_rcvd);
            fapi_eom_rcvd_bitmap = 0;
        }
    }
}
```

#### 2. **PHY Instance Worker Threads** (Per-Cell Processing)

**File**: `scf_5g_fapi_phy.cpp:352` (`on_msg()`)

**Thread Model**: One PHY instance per cell (up to `MAX_CELLS_PER_SLOT = 20`)

**Responsibilities**:
- Parse and validate PUSCH PDUs from UL_TTI.request
- Convert FAPI structures to internal slot command format
- Populate slot command circular buffer
- Register result callbacks for GPU processing completion

**Processing Flow**:
```cpp
bool phy::on_msg(nv_ipc_msg_t& msg) {
    // 1. Message validation
    sfn_slot_t& ss_msg = *(reinterpret_cast<sfn_slot_t*>(body_hdr.data));
    if (check_sfn_slot(msg.cell_id, typeID, ss_msg) < 0) {
        return ready_to_free; // Drop late messages
    }

    // 2. PUSCH PDU processing
    case SCF_FAPI_UL_TTI_REQUEST:
        on_ul_tti_request(ul_tti_req, msg);
        // -> on_pusch_pdu_info(pdu)
        // -> prepare_ul_slot_command(slot_ind, pdu)
        break;
}
```

#### 3. **UlPhyDriver Worker Thread Pool**

**File**: `nv_phy_driver_proxy.cpp` (PHY Driver layer)

**Thread Naming**: `UlPhyDriver04`, `UlPhyDriver05`, `UlPhyDriver06`, ...

**Responsibilities**:
- Execute GPU PUSCH processing operations
- Manage CUDA streams for async GPU execution
- Handle GPU memory transfers (H2D for config, D2H for results)
- Invoke cuPHY kernel launches

**Load Balancing**:
- **Round-robin slot distribution**: Slots assigned to workers in circular fashion
- **Slot advance**: Typically 20 slots ahead of current time
- **GPU stream parallelism**: Multiple streams per worker for overlapped execution

#### 4. **Auxiliary Threads**

**Cell Update Thread** (`cell_update_thread_func`):
- **File**: `nv_phy_module.cpp:764`
- **Purpose**: OAM-triggered cell configuration updates
- **Affinity**: Low-priority CPU core to avoid blocking critical paths

**SFN/Slot Sync Thread** (`sfn_slot_sync_cmd_thread_func`):
- **File**: `nv_phy_module.cpp:808`
- **Purpose**: Synchronization commands for multi-DU/UE scenarios

**Tick Generator Thread**:
- **File**: `nv_tick_generator.cpp`
- **Purpose**: Generate timing ticks for slot boundary synchronization

### Thread Synchronization Mechanisms

#### Atomic Variables
```cpp
// cuPHY-CP/cuphyl2adapter/lib/nvPHY/nv_phy_module.hpp:450
std::atomic<nanoseconds> current_tick_;   // Current slot time
std::atomic<sfn_slot_t> ss_tick;          // Current SFN/Slot
std::atomic<bool> sync_rcvd_from_ue;      // SE sync flags
```

#### Bitmaps for Multi-Cell Coordination
```cpp
// nv_phy_module.hpp:511-512
uint64_t active_cell_bitmap;     // Cells currently active (bit per cell)
uint64_t fapi_eom_rcvd_bitmap;   // End-of-message received per cell
```

**Synchronization Logic**:
```cpp
// Process slot command when all active cells complete
if (active_cell_bitmap &&
    (fapi_eom_rcvd_bitmap & active_cell_bitmap) == active_cell_bitmap) {
    process_phy_commands(slot_end_rcvd);
}
```

#### Mutexes
```cpp
std::mutex dl_tbs_lock;   // Protect DL TB queue
std::mutex dl_tti_lock;   // Protect DL TTI queue
std::mutex tick_lock;     // Protect tick updates
```

---

## Multi-Task Processing Architecture

### Slot Command Orchestration

The system uses a **slot command circular buffer** to decouple FAPI message reception from GPU processing:

```cpp
// cuPHY-CP/cuphyl2adapter/lib/nvPHY/nv_phy_module.hpp:527
std::vector<slot_command_api::slot_command> slot_command_array; // Size: 512
uint32_t current_slot_cmd_index; // Circular buffer index
```

#### Slot Command Structure

**File**: `scf_5g_slot_commands_common.hpp:1-435`

```cpp
struct slot_command {
    slot_indication slot_info;           // SFN, Slot, Tick
    std::array<cell_sub_command, MAX_CELLS_PER_SLOT> cells;
    cell_group_command cell_groups;      // Multi-cell coordination
};

struct cell_sub_command {
    // Uplink channels
    std::vector<pusch_params> pusch;     // PUSCH PDUs
    std::vector<pucch_params> pucch;     // PUCCH PDUs
    std::vector<srs_params> srs;         // SRS PDUs
    std::vector<prach_params> prach;     // PRACH PDUs

    // Downlink channels (for reference)
    std::vector<pdsch_params> pdsch;
    std::vector<pdcch_params> pdcch;
    std::vector<csirs_params> csirs;
};
```

#### PUSCH Parameters Structure

```cpp
struct pusch_params {
    // Resource allocation
    uint16_t rnti;
    uint16_t startPrbc;          // Starting PRB
    uint16_t numPrbc;            // Number of PRBs
    uint8_t startSymbolIndex;    // Starting OFDM symbol
    uint8_t numSymbols;          // Number of symbols

    // Modulation and coding
    uint8_t mcs_index;           // MCS index (0-31)
    uint8_t mcs_table;           // MCS table selection
    uint8_t rv_index;            // Redundancy version
    uint8_t harq_process_id;     // HARQ process ID
    uint32_t tb_size;            // Transport block size

    // DMRS configuration
    uint16_t ul_dmrs_sym_pos;    // DMRS symbol positions
    uint16_t dmrs_ports;         // DMRS port bitmap
    uint8_t dmrs_config_type;    // Type 1 or Type 2
    uint8_t num_dmrs_cdm_groups_no_data;
    uint16_t ul_dmrs_scrambling_id;
    uint8_t scid;                // Scrambling ID selection

    // Transform precoding (DFT-s-OFDM)
    uint8_t transform_precoding;

    // UCI on PUSCH
    uint16_t harq_ack_bit_length;
    uint16_t csi_part_1_bit_length;
    uint16_t csi_part_2_bit_length;
    uint8_t alpha_scaling;
    uint8_t beta_offset_harq_ack;
    uint8_t beta_offset_csi_1;
    uint8_t beta_offset_csi_2;

    // PT-RS (Phase Tracking Reference Signal)
    uint8_t ulPtrsSampleDensity;
    uint8_t ulPtrsTimeDensityTransformPrecoding;

    // Beamforming
    scf_fapi_rx_precoding_beamforming_t beamforming;

    // Power control
    int8_t ul_target_received_power;
};
```

### Task Processing Stages

#### Stage 1: FAPI Message Reception and Validation

**Location**: `scf_5g_fapi_phy.cpp:1799` (`on_pusch_pdu_info()`)

**Validation Checks**:
- RNTI validity (non-zero)
- BWP size and start position (0-275 PRBs)
- Subcarrier spacing (μ = 0-3)
- MCS index (0-31) and table (0-4)
- Transform precoding (0-1)
- Number of layers (1-4)
- DMRS configuration validity
- Resource allocation (RB start, RB size)
- UCI parameter ranges

**Error Handling**:
```cpp
#ifdef ENABLE_L2_SLT_RSP
auto& group_l1_limit = phy_module().get_group_limit_errors();
auto ret = validate_pusch_pdu_l1_limits(pdu, group_l1_limit.pusch_errors);
if (ret == INVALID_FAPI_PDU) {
    return; // Drop invalid PDU, send error indication
}
#endif
```

#### Stage 2: Slot Command Preparation

**Location**: `scf_5g_fapi_phy.cpp:2072` (`prepare_ul_slot_command()`)

**Tasks**:
1. **Retrieve slot command buffer**:
   ```cpp
   auto& slot_cmd = phy_module().cell_sub_command(get_carrier_id());
   auto grp_cmd = phy_module().group_command();
   ```

2. **Convert FAPI PDU to internal format**:
   - Map FAPI fields to `pusch_params` structure
   - Calculate derived parameters (e.g., DMRS positions)
   - Setup beamforming weights if mMIMO enabled

3. **PRB-level resource tracking**:
   - Update PRB symbol list for spectrum visualization
   - Track PRB usage for interference management

4. **Beamforming coefficient handling**:
   ```cpp
   if (get_mMIMO_enable_info()) {
       bfwCoeff_mem_info = phy_module().get_bfw_coeff_buff_info(
           get_carrier_id(),
           prv_slot_idx % MAX_BFW_COFF_STORE_INDEX
       );
   }
   ```

#### Stage 3: Slot Command Execution

**Location**: `nv_phy_module.cpp:614` (`process_phy_commands()`)

**Process Flow**:
```cpp
void PHY_module::process_phy_commands(bool slot_end_rcvd) {
    // 1. Update slot command index
    update_slot_cmds_indexes();
    current_slot_cmd_index = (current_slot_cmd_index + 1) % 512;

    // 2. Invoke callbacks for current slot
    send_call_backs();

    // 3. Dispatch to PHY driver
    slot_command& cmd = slot_command_array[current_slot_cmd_index];

    // PHY driver distributes to worker threads (UlPhyDriver##)
    // Workers invoke GPU PUSCH processing asynchronously
}
```

#### Stage 4: GPU Processing (Asynchronous)

**cuPHY Operations** (GPU kernels):
1. **Channel estimation**: Estimate channel from DMRS
2. **Equalization**: Equalize received signal
3. **LDPC decoding**: Decode transport block
4. **CRC checking**: Verify transport block integrity
5. **UCI extraction**: Extract HARQ-ACK, CSI feedback

**Memory Transfers**:
- H2D: PUSCH configuration parameters
- D2H: Decoded transport blocks, CRC results, UCI bits

#### Stage 5: Result Callback Processing

**Location**: `scf_5g_fapi_phy.cpp:4596-4612`

**Callback Sequence**:
```cpp
// Registered callbacks invoked upon GPU completion
callbacks.ul_callback = [this](slot_indication& slot,
                                const pusch_params& params,
                                cuphyPuschDataOut_t* out,
                                cuphyPuschStatPrms_t* puschStatPrms) {

    // 1. Send CRC indication
    auto crcFails = send_crc_indication(slot, params, out, puschStatPrms);

    // 2. Send RX_DATA indication (decoded TB)
    send_rx_data_indication(slot, params, out, puschStatPrms);

    // 3. Send UCI indication (HARQ-ACK, CSI on PUSCH)
    if (params.harq_ack_bit_length || params.csi_part_1_bit_length) {
        send_uci_indication(slot, params, *out, puschStatPrms);
    }

    // 4. Optional: RX_PE_NOISE_VAR indication
    send_rx_pe_noise_var_indication(slot, params, out, puschStatPrms);
};
```

---

## PUSCH Processing Pipeline

### End-to-End Data Flow

```
┌──────────────────────────────────────────────────────────────────────┐
│ MAC Layer                                                             │
│ • Generates UL_TTI.request with PUSCH PDUs                           │
│ • Sends via IPC (shared memory queue)                                │
└──────────────┬───────────────────────────────────────────────────────┘
               │
               ▼
┌──────────────────────────────────────────────────────────────────────┐
│ msg_processing thread (PHY_module)                              [T1] │
│ • epoll waits for IPC message availability                           │
│ • Reads message from transport layer                                 │
│ • Extracts SFN/Slot from message header                              │
└──────────────┬───────────────────────────────────────────────────────┘
               │
               ▼
┌──────────────────────────────────────────────────────────────────────┐
│ Message Dispatch (PHY_module)                                   [T1] │
│ • Check SFN/Slot validity (check_sfn_slot)                           │
│ • Route to appropriate PHY instance based on cell_id                 │
│ • Update fapi_eom_rcvd_bitmap for slot synchronization               │
└──────────────┬───────────────────────────────────────────────────────┘
               │
               ▼
┌──────────────────────────────────────────────────────────────────────┐
│ scf_5g_fapi::phy::on_msg()                                      [T1] │
│ • Parse FAPI message type (UL_TTI.request)                           │
│ • Extract PUSCH PDU from message payload                             │
│ • Call on_pusch_pdu_info(pdu)                                        │
└──────────────┬───────────────────────────────────────────────────────┘
               │
               ▼
┌──────────────────────────────────────────────────────────────────────┐
│ PUSCH PDU Validation (on_pusch_pdu_info)                        [T1] │
│ • Validate RNTI, BWP, MCS, DMRS parameters                           │
│ • Check L1 resource limits (if ENABLE_L2_SLT_RSP)                    │
│ • Return error if validation fails                                   │
└──────────────┬───────────────────────────────────────────────────────┘
               │
               ▼
┌──────────────────────────────────────────────────────────────────────┐
│ Slot Command Preparation (prepare_ul_slot_command)             [T1] │
│ • Get slot command buffer: slot_command_array[current_slot_cmd_index]│
│ • Convert FAPI PUSCH PDU → pusch_params                              │
│ • Add to slot_cmd.cells[cell_id].pusch vector                        │
│ • Update PRB/symbol resource maps                                    │
│ • Handle beamforming coefficients (mMIMO)                            │
└──────────────┬───────────────────────────────────────────────────────┘
               │
               ▼
┌──────────────────────────────────────────────────────────────────────┐
│ Slot Synchronization (PHY_module)                              [T1] │
│ • Wait for all active cells to complete (bitmap check)               │
│ • When ready: process_phy_commands(slot_end_rcvd)                    │
└──────────────┬───────────────────────────────────────────────────────┘
               │
               ▼
┌──────────────────────────────────────────────────────────────────────┐
│ Slot Command Execution (process_phy_commands)                  [T1] │
│ • Increment current_slot_cmd_index (circular buffer)                 │
│ • Invoke send_call_backs() → triggers PHY driver processing          │
└──────────────┬───────────────────────────────────────────────────────┘
               │
               ▼
┌──────────────────────────────────────────────────────────────────────┐
│ PHY Driver Dispatch                                          [T2-TN] │
│ • UlPhyDriver worker threads pick up slot commands                   │
│ • Round-robin distribution across worker pool                        │
│ • Each worker manages GPU streams                                    │
└──────────────┬───────────────────────────────────────────────────────┘
               │
               ▼
┌──────────────────────────────────────────────────────────────────────┐
│ GPU PUSCH Processing (cuPHY)                                   [GPU] │
│ • H2D transfer: Copy PUSCH config to GPU                             │
│ • Channel estimation from DMRS                                       │
│ • Equalization (MMSE/ZF)                                             │
│ • Demodulation (QPSK/16QAM/64QAM/256QAM)                             │
│ • LDPC decoding                                                      │
│ • CRC checking                                                       │
│ • UCI extraction (HARQ-ACK, CSI)                                     │
│ • D2H transfer: Copy results to CPU                                  │
└──────────────┬───────────────────────────────────────────────────────┘
               │
               ▼
┌──────────────────────────────────────────────────────────────────────┐
│ Result Callbacks                                             [T2-TN] │
│ • GPU completion triggers registered callback                        │
│ • cuphyPuschDataOut_t contains decoded data + CRC results            │
└──────────────┬───────────────────────────────────────────────────────┘
               │
               ▼
┌──────────────────────────────────────────────────────────────────────┐
│ Send CRC Indication (send_crc_indication)                    [T2-TN] │
│ • Check CRC results for each transport block                         │
│ • Build CRC.indication FAPI message                                  │
│ • Send to MAC via IPC                                                │
└──────────────┬───────────────────────────────────────────────────────┘
               │
               ▼
┌──────────────────────────────────────────────────────────────────────┐
│ Send RX_DATA Indication (send_rx_data_indication)           [T2-TN] │
│ • Extract decoded transport block from GPU output                    │
│ • Build RX_DATA.indication FAPI message                              │
│ • Attach decoded data payload                                        │
│ • Send to MAC via IPC                                                │
└──────────────┬───────────────────────────────────────────────────────┘
               │
               ▼
┌──────────────────────────────────────────────────────────────────────┐
│ Send UCI Indication (send_uci_indication)                   [T2-TN] │
│ • Extract HARQ-ACK bits                                              │
│ • Extract CSI Part 1/Part 2 bits                                     │
│ • Build UCI.indication FAPI message                                  │
│ • Send to MAC via IPC                                                │
└──────────────┬───────────────────────────────────────────────────────┘
               │
               ▼
┌──────────────────────────────────────────────────────────────────────┐
│ MAC Layer                                                             │
│ • Receives CRC.indication, RX_DATA.indication, UCI.indication        │
│ • Processes decoded uplink data                                      │
│ • Updates HARQ processes based on CRC results                        │
│ • Processes CSI feedback for scheduling                              │
└──────────────────────────────────────────────────────────────────────┘

Thread Key:
[T1]    = msg_processing thread
[T2-TN] = UlPhyDriver worker threads
[GPU]   = GPU execution (asynchronous)
```

---

## Key Data Structures

### 1. PHY Module Control Structure

**File**: `nv_phy_module.hpp:115-546`

```cpp
class PHY_module {
private:
    // Transport layer
    phy_mac_transport_wrapper transport_wrapper_;

    // Worker instances (one per cell)
    std::vector<PHY_instance_ptr> phy_instances_;  // Actual objects
    std::vector<PHY_instance_ref> phy_refs_;       // References for API

    // Slot command circular buffer (512 slots)
    std::vector<slot_command_api::slot_command> slot_command_array;
    uint32_t current_slot_cmd_index;

    // Multi-cell synchronization
    uint num_cells_active;
    uint32_t total_cell_num;
    uint64_t active_cell_bitmap;       // Bit per cell
    uint64_t fapi_eom_rcvd_bitmap;     // End-of-message bitmap

    // Timing synchronization
    std::atomic<nanoseconds> current_tick_;
    nv::TickUpdater tick_updater_;

    // L1 limit error tracking
    #ifdef ENABLE_L2_SLT_RSP
    std::array<slot_limit_cell_error_t, MAX_CELLS_PER_SLOT> cell_limit_errors_;
    slot_limit_group_error_t group_limit_errors_;
    #endif

    // Configuration
    phy_config_option config_options_;
};
```

### 2. FAPI PUSCH PDU Structure

**File**: `scf_5g_fapi.h`

```cpp
typedef struct scf_fapi_pusch_pdu_t {
    // Basic identification
    uint16_t rnti;
    uint32_t handle;

    // BWP configuration
    struct {
        uint16_t bwp_size;        // 1-275 PRBs
        uint16_t bwp_start;       // 0-274
        uint8_t scs;              // Subcarrier spacing: 0=15kHz, 1=30kHz, 2=60kHz, 3=120kHz
        uint8_t cyclic_prefix;
    } bwp;

    // Modulation and coding
    uint8_t qam_mod_order;         // 2=QPSK, 4=16QAM, 6=64QAM, 8=256QAM
    uint8_t mcs_index;             // 0-31
    uint8_t mcs_table;             // 0-4
    uint8_t transform_precoding;   // 0=disabled, 1=enabled (DFT-s-OFDM)
    uint16_t data_scrambling_id;
    uint8_t num_of_layers;         // 1-4

    // DMRS configuration
    uint16_t ul_dmrs_sym_pos;      // Bitmap of DMRS symbol positions
    uint8_t dmrs_config_type;      // 0=Type1, 1=Type2
    uint16_t ul_dmrs_scrambling_id;
    uint8_t scid;                  // 0 or 1
    uint8_t num_dmrs_cdm_groups_no_data;  // 1-3
    uint16_t dmrs_ports;           // Bitmap of DMRS ports

    // Resource allocation
    uint8_t resource_alloc;        // 0=Type0, 1=Type1
    uint8_t rb_bitmap[36];         // For Type 0
    uint16_t rb_start;             // For Type 1
    uint16_t rb_size;              // For Type 1
    uint8_t vrb_to_prb_mapping;
    uint8_t frequency_hopping;
    uint16_t tx_direct_current_location;
    uint8_t ul_frequency_shift_7p5_khz;

    // Time domain allocation
    uint8_t start_symbol_index;    // 0-13
    uint8_t num_of_symbols;        // 1-14

    // PUSCH identity
    uint16_t pusch_identity;       // 0-1007

    // PDU bitmap (indicates which optional parts are present)
    uint8_t pdu_bitmap;
    // Bit 0: puschData present
    // Bit 1: puschUci present
    // Bit 2: puschPtrs present
    // Bit 3: dftsOfdm present

    // Beamforming
    scf_fapi_rx_precoding_beamforming_t beamforming;

    // Variable payload follows
    uint8_t payload[];

    // Optional: scf_fapi_pusch_data_t (if bit 0 set)
    // Optional: scf_fapi_pusch_uci_t (if bit 1 set)
    // Optional: scf_fapi_pusch_ptrs_t (if bit 2 set)
    // Optional: scf_fapi_pusch_dftsofdm_t (if bit 3 set)
} scf_fapi_pusch_pdu_t;
```

### 3. PUSCH Data Sub-structure

```cpp
typedef struct scf_fapi_pusch_data_t {
    uint8_t rv_index;              // Redundancy version: 0-3
    uint8_t harq_process_id;       // 0-15
    uint8_t new_data_indicator;    // 0 or 1
    uint32_t tb_size;              // Transport block size in bytes
    uint16_t num_cb;               // Number of code blocks
    uint8_t cb_present_and_position[];  // Bitmap of CB retransmissions
} scf_fapi_pusch_data_t;
```

### 4. PUSCH UCI Sub-structure

```cpp
typedef struct scf_fapi_pusch_uci_t {
    uint16_t harq_ack_bit_length;      // 0-1706 bits
    uint16_t csi_part_1_bit_length;    // 0-1706 bits

    #ifdef SCF_FAPI_10_04
    uint16_t flag_csi_part2;           // 0 or 0xFFFF
    #else
    uint16_t csi_part_2_bit_length;    // 0-1706 bits
    #endif

    uint8_t alpha_scaling;             // 0-3
    uint8_t beta_offset_harq_ack;      // 0-15
    uint8_t beta_offset_csi_1;         // 0-15
    uint8_t beta_offset_csi_2;         // 0-15
} scf_fapi_pusch_uci_t;
```

### 5. cuPHY PUSCH Output Structure

**File**: `cuphy.hpp` (cuPHY SDK)

```cpp
typedef struct cuphyPuschDataOut_t {
    // Decoded transport block
    uint8_t* tbData;               // Pointer to decoded TB
    uint32_t tbSize;               // TB size in bytes

    // CRC results
    uint32_t crcResult;            // 0=pass, 1=fail

    // UCI on PUSCH
    uint8_t* harqAckBits;          // HARQ-ACK bits
    uint16_t harqAckLength;
    uint8_t* csiPart1Bits;         // CSI Part 1 bits
    uint16_t csiPart1Length;
    uint8_t* csiPart2Bits;         // CSI Part 2 bits
    uint16_t csiPart2Length;

    // Channel quality metrics
    float sinr;                    // SINR estimate
    float rsrp;                    // RSRP (dBm)
    float rssi;                    // RSSI (dBm)
    int16_t timing_advance;        // TA estimate

    // Noise and power estimates
    float noise_variance;
    float received_power;
} cuphyPuschDataOut_t;
```

### 6. Slot Limit Error Structures

**File**: `nv_phy_limit_errors.hpp:1-29`

```cpp
typedef struct pusch_limit_error_t {
    uint8_t errors;    // L1 limit violation error code
    uint8_t parsed;    // Validation status: 0=not parsed, 1=parsed successfully
} pusch_limit_error_t;

typedef struct slot_limit_cell_error_t {
    pusch_limit_error_t pusch_errors;
    pucch_limit_error_t pucch_errors;
    srs_limit_error_t srs_errors;
    prach_limit_error_t prach_errors;
} slot_limit_cell_error_t;

typedef struct slot_limit_group_error_t {
    pusch_limit_error_t pusch_errors;
    pucch_limit_error_t pucch_errors;
    pdsch_limit_error_t pdsch_errors;
    pdcch_limit_error_t pdcch_errors;
} slot_limit_group_error_t;
```

---

## Detailed Component Analysis

### Component 1: PHY_module (Main Orchestrator)

**File**: `nv_phy_module.hpp` / `nv_phy_module.cpp`

**Key Methods**:

#### `msg_processing()`
- **Purpose**: Main event loop for FAPI message reception
- **Synchronization**: Uses `epoll` for efficient IPC event handling
- **Flow**:
  1. Wait for IPC message via `epoll_ctx_p->add_fd()`
  2. Receive message: `transport_wrapper().rx_recv(smsg)`
  3. Dispatch to PHY instance: `phy_refs_[smsg.cell_id].get().on_msg(smsg)`
  4. Check slot completion: bitmap comparison
  5. Trigger processing: `process_phy_commands()`

#### `process_phy_commands(bool slot_end_rcvd)`
- **Purpose**: Execute slot commands and advance circular buffer
- **Location**: `nv_phy_module.cpp:614`
- **Flow**:
  ```cpp
  void PHY_module::process_phy_commands(bool slot_end_rcvd) {
      // Advance circular buffer index
      update_slot_cmds_indexes();

      // Invoke callbacks (triggers GPU processing)
      send_call_backs();

      // Reset slot state
      new_slot_ = true;
      is_ul_slot_ = false;
      is_dl_slot_ = false;
  }
  ```

#### `tick_received(nanoseconds& tick)`
- **Purpose**: Synchronize slot boundaries across cells
- **Mechanism**: Atomic tick updates with slot advance calculation

### Component 2: scf_5g_fapi::phy (Per-Cell Worker)

**File**: `scf_5g_fapi_phy.hpp` / `scf_5g_fapi_phy.cpp`

**Key Methods**:

#### `on_msg(nv_ipc_msg_t& msg)`
**Location**: `scf_5g_fapi_phy.cpp:352`

```cpp
bool phy::on_msg(nv_ipc_msg_t& msg) {
    rx_msg_reader reader(msg);
    for (rx_msg_reader::iterator it = reader.begin(); it != reader.end(); ++it) {
        scf_fapi_body_header_t& body_hdr = (*it);
        uint8_t typeID = body_hdr.type_id;

        switch (typeID) {
            case SCF_FAPI_CONFIG_REQUEST:
                on_config_request(...);
                break;
            case SCF_FAPI_START_REQUEST:
                on_cell_start_request(msg.cell_id);
                break;
            case SCF_FAPI_STOP_REQUEST:
                on_cell_stop_request(msg.cell_id);
                break;
            case SCF_FAPI_DL_TTI_REQUEST:
                on_dl_tti_request(dl_tti_req, msg, pdsch_valid_flag);
                break;
            case SCF_FAPI_UL_TTI_REQUEST:
                on_ul_tti_request(ul_tti_req, msg);
                break;
            case SCF_FAPI_TX_DATA_REQUEST:
                on_phy_dl_tx_request(tx_data_req, msg, pdsch_valid_flag);
                break;
            case SCF_FAPI_UL_DCI_REQUEST:
                on_ul_dci_request(ul_dci_req, msg);
                break;
        }
    }
    return ready_to_free;
}
```

#### `on_ul_tti_request(scf_fapi_ul_tti_req_t& request, nv_ipc_msg_t& ipc_msg)`
**Location**: `scf_5g_fapi_phy.cpp:1400`

```cpp
void phy::on_ul_tti_request(scf_fapi_ul_tti_req_t& request, nv_ipc_msg_t& ipc_msg) {
    slot_command_api::slot_indication slot_ind(
        request.sfn,
        request.slot,
        phy_module().get_mu_highest()
    );

    // Iterate through PDUs in UL_TTI.request
    for (uint16_t idx = 0; idx < request.npdus; idx++) {
        scf_fapi_ul_tti_req_pdu_t& pdu = request.pdus[idx];

        switch (pdu.pdu_type) {
            case SCF_FAPI_UL_TTI_PDU_TYPE_PRACH:
                on_prach_pdu_info(pdu.prach_pdu, slot_ind);
                break;
            case SCF_FAPI_UL_TTI_PDU_TYPE_PUCCH:
                on_pucch_pdu_info(pdu.pucch_pdu, slot_ind);
                break;
            case SCF_FAPI_UL_TTI_PDU_TYPE_PUSCH:
                if (on_pusch_pdu_info(pdu.pusch_pdu)) {
                    prepare_ul_slot_command(slot_ind, pdu.pusch_pdu);
                }
                break;
            case SCF_FAPI_UL_TTI_PDU_TYPE_SRS:
                on_srs_pdu_info(pdu.srs_pdu, slot_ind, ...);
                break;
        }
    }
}
```

#### `on_pusch_pdu_info(scf_fapi_pusch_pdu_t& pdu_info)`
**Location**: `scf_5g_fapi_phy.cpp:1799`

- **Purpose**: Validate PUSCH PDU parameters against spec limits
- **Returns**: `true` if validation passes, `false` otherwise
- **Validation Examples**:
  - RNTI must be non-zero
  - BWP size: 1-275 PRBs
  - MCS index: 0-31
  - Number of layers: 1-4
  - DMRS symbol positions must be valid
  - Resource allocation must be in bounds

#### `prepare_ul_slot_command(slot_indication& slot_ind, scf_fapi_pusch_pdu_t& pdu)`
**Location**: `scf_5g_fapi_phy.cpp:2072`

```cpp
void phy::prepare_ul_slot_command(slot_indication& slot_ind, scf_fapi_pusch_pdu_t& pdu) {
    // 1. Get slot command buffer
    auto& slot_cmd = phy_module().cell_sub_command(get_carrier_id());

    // 2. Create pusch_params entry
    pusch_params params;

    // 3. Map FAPI fields to internal format
    params.rnti = pdu.rnti;
    params.startPrbc = pdu.rb_start;
    params.numPrbc = pdu.rb_size;
    params.startSymbolIndex = pdu.start_symbol_index;
    params.numSymbols = pdu.num_of_symbols;
    params.mcs_index = pdu.mcs_index;
    params.mcs_table = pdu.mcs_table;
    // ... (many more field mappings)

    // 4. Parse optional sub-structures
    if (pdu.pdu_bitmap & 0x01) {  // puschData present
        auto* data = reinterpret_cast<scf_fapi_pusch_data_t*>(&pdu.payload[0]);
        params.rv_index = data->rv_index;
        params.harq_process_id = data->harq_process_id;
        params.tb_size = data->tb_size;
    }

    if (pdu.pdu_bitmap & 0x02) {  // puschUci present
        auto* uci = reinterpret_cast<scf_fapi_pusch_uci_t*>(&pdu.payload[offset]);
        params.harq_ack_bit_length = uci->harq_ack_bit_length;
        params.csi_part_1_bit_length = uci->csi_part_1_bit_length;
        params.beta_offset_harq_ack = uci->beta_offset_harq_ack;
        // ...
    }

    // 5. Add to slot command
    slot_cmd.pusch.push_back(params);

    // 6. Update resource tracking
    update_prb_sym_list(slot_info, prb_index,
                        params.startSymbolIndex, params.numSymbols,
                        channel_type::PUSCH, ru_type);
}
```

#### `send_crc_indication()`
**Location**: `scf_5g_fapi_phy.cpp:4722`

```cpp
uint16_t phy::send_crc_indication(
    const slot_indication& slot,
    const pusch_params& params,
    cuphyPuschDataOut_t const* out,
    cuphyPuschStatPrms_t const* puschStatPrms)
{
    // Allocate FAPI message buffer
    scf_fapi_crc_ind_t* crc_ind = allocate_crc_indication();

    // Fill header
    crc_ind->sfn = slot.sfn_;
    crc_ind->slot = slot.slot_;
    crc_ind->num_crcs = 1;

    // Fill CRC PDU
    auto& crc_pdu = crc_ind->crcs[0];
    crc_pdu.handle = params.handle;
    crc_pdu.rnti = params.rnti;
    crc_pdu.harq_id = params.harq_process_id;
    crc_pdu.tb_crc_status = (out->crcResult == 0) ? 0 : 1;  // 0=pass, 1=fail
    crc_pdu.num_cb = out->num_cb;
    crc_pdu.timing_advance = calculate_timing_advance(out->timing_advance_estimate);
    crc_pdu.rssi = calculate_rssi(out->rssi);

    // Send to MAC
    phy_module().transport(get_carrier_id()).tx_send(crc_ind);

    // Update statistics
    if (crc_pdu.tb_crc_status) {
        ul_crc_err[get_carrier_id()]++;
        ul_crc_err_total[get_carrier_id()]++;
    }

    return crc_pdu.tb_crc_status;
}
```

#### `send_rx_data_indication()`
**Location**: `scf_5g_fapi_phy.cpp:4868`

```cpp
void phy::send_rx_data_indication(
    const slot_indication& slot,
    const pusch_params& params,
    cuphyPuschDataOut_t const* out,
    cuphyPuschStatPrms_t const* puschStatPrms)
{
    // Allocate message with space for decoded data
    scf_fapi_rx_data_ind_t* rx_data = allocate_rx_data_indication(out->tbSize);

    // Fill header
    rx_data->sfn = slot.sfn_;
    rx_data->slot = slot.slot_;
    rx_data->num_pdus = 1;

    // Fill PDU
    auto& pdu = rx_data->pdus[0];
    pdu.handle = params.handle;
    pdu.rnti = params.rnti;
    pdu.harq_id = params.harq_process_id;
    pdu.pdu_length = out->tbSize;

    // Copy decoded transport block data
    memcpy(pdu.pdu_data, out->tbData, out->tbSize);

    // Send to MAC
    phy_module().transport(get_carrier_id()).tx_send(rx_data);

    // Update throughput statistics
    ul_thrput[get_carrier_id()] += out->tbSize;
}
```

#### `send_uci_indication()`
**Location**: `scf_5g_fapi_phy.cpp:4143`

```cpp
void phy::send_uci_indication(
    const slot_indication& slot,
    const pusch_params& params,
    const cuphyPuschDataOut_t& out,
    cuphyPuschStatPrms_t const* puschStatPrms)
{
    // Check if UCI is present
    if (params.harq_ack_bit_length == 0 && params.csi_part_1_bit_length == 0) {
        return;  // No UCI on this PUSCH
    }

    // Allocate UCI indication message
    scf_fapi_uci_ind_t* uci_ind = allocate_uci_indication();

    // Fill header
    uci_ind->sfn = slot.sfn_;
    uci_ind->slot = slot.slot_;
    uci_ind->num_ucis = 1;

    // Fill UCI PDU
    auto& uci_pdu = uci_ind->uci_pdus[0];
    uci_pdu.pdu_type = SCF_FAPI_UCI_IND_PUSCH;
    uci_pdu.rnti = params.rnti;

    auto& pusch_uci = uci_pdu.pusch_uci;
    pusch_uci.ul_sinr_metric = convert_sinr(out.sinr);
    pusch_uci.timing_advance = calculate_ta(out.timing_advance_estimate);
    pusch_uci.rssi = calculate_rssi(out.rssi);
    pusch_uci.rsrp = calculate_rsrp(out.rsrp);

    // HARQ-ACK
    if (params.harq_ack_bit_length > 0) {
        pusch_uci.harq_payload_present = 1;
        pusch_uci.harq_bit_len = params.harq_ack_bit_length;
        memcpy(pusch_uci.harq_payload, out.harqAckBits,
               (params.harq_ack_bit_length + 7) / 8);
    }

    // CSI Part 1
    if (params.csi_part_1_bit_length > 0) {
        pusch_uci.csi_part1_payload_present = 1;
        pusch_uci.csi_part1_bit_len = params.csi_part_1_bit_length;
        memcpy(pusch_uci.csi_part1_payload, out.csiPart1Bits,
               (params.csi_part_1_bit_length + 7) / 8);
    }

    // CSI Part 2
    if (params.csi_part_2_bit_length > 0) {
        pusch_uci.csi_part2_payload_present = 1;
        pusch_uci.csi_part2_bit_len = params.csi_part_2_bit_length;
        memcpy(pusch_uci.csi_part2_payload, out.csiPart2Bits,
               (params.csi_part_2_bit_length + 7) / 8);
    }

    // Send to MAC
    phy_module().transport(get_carrier_id()).tx_send(uci_ind);
}
```

### Component 3: PHY Driver Proxy

**File**: `nv_phy_driver_proxy.hpp` / `nv_phy_driver_proxy.cpp`

**Purpose**: Interface between L2 adapter and cuPHY driver

**Key Responsibilities**:
- Manage worker thread pool (UlPhyDriver##)
- Distribute slot commands to workers
- Handle GPU resource management
- Coordinate callback registration

**Thread Pool Management**:
```cpp
// Worker thread naming convention
const char* worker_names[] = {
    "UlPhyDriver04",
    "UlPhyDriver05",
    "UlPhyDriver06",
    // ... up to num_workers
};

// Round-robin slot assignment
uint worker_index = (slot.sfn_ * slots_per_frame + slot.slot_) % num_workers;
worker_pool[worker_index]->process_slot_command(slot_cmd);
```

---

## Processing Flow Diagrams

### Diagram 1: Message Flow from MAC to GPU

```
┌─────────────┐
│  MAC Layer  │
│  UL Sched   │
└──────┬──────┘
       │ UL_TTI.request
       │ (PUSCH PDU)
       ▼
┌─────────────────────────────────────┐
│  IPC Transport (Shared Memory)      │
│  - Message queues per cell          │
└──────┬──────────────────────────────┘
       │ epoll event
       ▼
┌─────────────────────────────────────┐
│  msg_processing thread              │
│  PHY_module::msg_processing()       │
│  - Read from IPC                    │
│  - Dispatch to cell instance        │
└──────┬──────────────────────────────┘
       │ Cell-specific routing
       ▼
┌─────────────────────────────────────┐
│  scf_5g_fapi::phy::on_msg()         │
│  - Parse FAPI message               │
│  - Extract PUSCH PDU                │
└──────┬──────────────────────────────┘
       │
       ▼
┌─────────────────────────────────────┐
│  on_pusch_pdu_info()                │
│  - Validate parameters              │
│  - Check L1 limits                  │
└──────┬──────────────────────────────┘
       │ Validation passed
       ▼
┌─────────────────────────────────────┐
│  prepare_ul_slot_command()          │
│  - Convert FAPI → pusch_params      │
│  - Add to slot_command_array        │
│  - Update PRB maps                  │
└──────┬──────────────────────────────┘
       │
       ▼
┌─────────────────────────────────────┐
│  Slot Synchronization               │
│  - Wait for all active cells        │
│  - Bitmap: fapi_eom_rcvd_bitmap     │
└──────┬──────────────────────────────┘
       │ All cells ready
       ▼
┌─────────────────────────────────────┐
│  process_phy_commands()             │
│  - Advance circular buffer          │
│  - Trigger callbacks                │
└──────┬──────────────────────────────┘
       │
       ▼
┌─────────────────────────────────────┐
│  PHY Driver Worker Selection        │
│  - Round-robin by slot number       │
│  - UlPhyDriver## picks up command   │
└──────┬──────────────────────────────┘
       │
       ▼
┌─────────────────────────────────────┐
│  GPU PUSCH Processing               │
│  cuPHY kernels:                     │
│  1. Channel estimation              │
│  2. Equalization                    │
│  3. Demodulation                    │
│  4. LDPC decoding                   │
│  5. CRC check                       │
│  6. UCI extraction                  │
└──────┬──────────────────────────────┘
       │ GPU completion
       ▼
┌─────────────────────────────────────┐
│  Result Callbacks                   │
│  - send_crc_indication()            │
│  - send_rx_data_indication()        │
│  - send_uci_indication()            │
└──────┬──────────────────────────────┘
       │ FAPI indications
       ▼
┌─────────────────────────────────────┐
│  IPC Transport (to MAC)             │
└──────┬──────────────────────────────┘
       │
       ▼
┌─────────────┐
│  MAC Layer  │
│  UL Data    │
└─────────────┘
```

### Diagram 2: Multi-Cell Slot Synchronization

```
Time →

Cell 0:  [UL_TTI.req]──┐
                       │
Cell 1:  [UL_TTI.req]──┤
                       │
Cell 2:  [UL_TTI.req]──┼──▶ fapi_eom_rcvd_bitmap |= (1 << cell_id)
                       │
Cell 3:  [UL_TTI.req]──┘

                       ▼
            ┌──────────────────────┐
            │ Bitmap Check:        │
            │ (fapi_eom_rcvd_bitmap│
            │  & active_cell_bitmap│
            │ == active_cell_bitmap)│
            └──────────┬───────────┘
                       │ All cells ready
                       ▼
            ┌──────────────────────┐
            │ process_phy_commands()│
            │ - Execute slot       │
            │ - Clear bitmap       │
            └──────────────────────┘
```

### Diagram 3: Circular Buffer Management

```
slot_command_array[512]  (Circular Buffer)

Current slot: SFN=100, Slot=5
Slot advance: 20 slots

Index 0   ──▶ SFN=100, Slot=5  ◀── current_slot_cmd_index
Index 1   ──▶ SFN=100, Slot=6
Index 2   ──▶ SFN=100, Slot=7
...
Index 20  ──▶ SFN=101, Slot=5  ◀── Being populated by msg_processing
...
Index 511 ──▶ SFN=151, Slot=1

When process_phy_commands() called:
1. current_slot_cmd_index++ (mod 512)
2. send_call_backs() on slot_command_array[current_slot_cmd_index]
3. GPU processes slot asynchronously
4. Results return via callbacks
```

---

## Thread Synchronization

### Synchronization Mechanisms Summary

| Mechanism | Type | Purpose | Location |
|-----------|------|---------|----------|
| `active_cell_bitmap` | Atomic bitmap | Track active cells | `nv_phy_module.hpp:511` |
| `fapi_eom_rcvd_bitmap` | Atomic bitmap | Track EOM receipt | `nv_phy_module.hpp:512` |
| `current_tick_` | `std::atomic<nanoseconds>` | Tick synchronization | `nv_phy_module.hpp:450` |
| `dl_tbs_lock` | `std::mutex` | Protect DL TB queue | `nv_phy_module.hpp:438` |
| `tick_lock` | `std::mutex` | Protect tick updates | `nv_phy_module.hpp:449` |
| `epoll` | Event polling | IPC event notification | `nv_phy_module.cpp:218` |

### Slot Synchronization Algorithm

**Problem**: Ensure all cells complete FAPI message processing before executing slot command.

**Solution**: Bitmap-based barrier synchronization

```cpp
// Step 1: When UL_TTI.request received for a cell
void PHY_module::msg_processing() {
    // ... message processing ...

    // Mark this cell as having completed FAPI messages for current slot
    fapi_eom_rcvd_bitmap |= (1ULL << cell_id);

    // Step 2: Check if all active cells have completed
    if (active_cell_bitmap &&
        (fapi_eom_rcvd_bitmap & active_cell_bitmap) == active_cell_bitmap) {

        // Step 3: All cells ready, process the slot
        process_phy_commands(slot_end_rcvd);

        // Step 4: Reset bitmap for next slot
        fapi_eom_rcvd_bitmap = 0;
    }
}
```

**Example**:
- 4 cells active: `active_cell_bitmap = 0b1111`
- Cell 0 completes: `fapi_eom_rcvd_bitmap = 0b0001`
- Cell 1 completes: `fapi_eom_rcvd_bitmap = 0b0011`
- Cell 2 completes: `fapi_eom_rcvd_bitmap = 0b0111`
- Cell 3 completes: `fapi_eom_rcvd_bitmap = 0b1111` → **Trigger slot processing**

---

## Performance Considerations

### 1. Latency Optimization

**Slot Advance**: Typically 20 slots (~10ms @ 30kHz SCS)
- Provides sufficient time for GPU processing
- Allows overlapped execution across multiple slots
- Configurable via `tick_updater_.slot_advance_`

**Early UCI Indication**:
- Option to send UCI separately from RX_DATA for faster HARQ feedback
- Method: `send_early_uci_indication()` in `scf_5g_fapi_phy.hpp:122`

### 2. Throughput Optimization

**Multi-Cell Parallelism**:
- Up to 20 cells processed concurrently
- Each cell has dedicated PHY instance (worker)
- GPU streams allow parallel execution

**GPU Stream Management**:
- Multiple CUDA streams per worker thread
- Overlapped H2D, kernel execution, D2H
- Asynchronous callback invocation

### 3. Memory Management

**Zero-Copy IPC**:
- Shared memory between MAC and PHY
- Avoids data copying for large transport blocks
- `dl_tb_location` configurable: inline, host buffer, or GPU buffer

**Circular Buffer**:
- Fixed 512-slot buffer eliminates dynamic allocation
- Lock-free access via atomic index increment

**Pre-allocated Queues**:
```cpp
// nv_phy_module.cpp:154
dl_tbs_queue_(MAX_CELLS_PER_SLOT * 20)  // 20 slots worth of TBs
```

### 4. CPU Affinity and Priority

**Critical Threads** (High priority, isolated cores):
- `msg_processing`: Core affinity configurable via YAML
- `UlPhyDriver##`: Managed by PHY driver

**Non-Critical Threads** (Low priority cores):
- `cell_update_thread_func`: OAM updates
- `sfn_slot_sync_cmd_thread_func`: SE sync

**Configuration Example**:
```yaml
message_thread_config:
  name: "msg_processing"
  cpu_affinity: 4        # Isolated core
  sched_priority: 90     # High RT priority
```

### 5. Error Handling and Recovery

**L1 Limit Validation** (ENABLE_L2_SLT_RSP):
- Validate PUSCH PDU against L1 resource limits
- Early rejection prevents GPU resource waste
- Error indications sent back to MAC

**Late Message Handling**:
```cpp
if (check_sfn_slot(msg.cell_id, typeID, ss_msg) < 0) {
    send_error_indication(msg_id, SCF_ERROR_CODE_SFN_OUT_OF_SYNC, ...);
    return ready_to_free;  // Drop message
}
```

**CRC Failure Statistics**:
```cpp
std::atomic<uint32_t> ul_crc_err[MAX_CELLS_PER_SLOT];
std::atomic<uint32_t> ul_crc_err_total[MAX_CELLS_PER_SLOT];
```

---

## Conclusion

The NVIDIA Aerial cuBB PUSCH uplink processing architecture demonstrates a highly optimized, multi-layered design for 5G NR uplink data handling. Key architectural strengths include:

1. **Scalable Multi-Threading**: Hierarchical thread model supports up to 20 concurrent cells
2. **Efficient Synchronization**: Bitmap-based slot barriers minimize overhead
3. **GPU Acceleration**: Asynchronous cuPHY processing maximizes throughput
4. **Robust Validation**: Multi-stage PUSCH PDU validation ensures spec compliance
5. **Low Latency**: Circular buffer and slot advance enable predictable processing times

This architecture is well-suited for high-performance 5G base station deployments requiring maximum throughput and minimal latency.

---

## File Reference Index

### Core Architecture Files
- `nv_phy_module.hpp:115-546` - Main PHY module orchestrator
- `nv_phy_module.cpp:150-850` - PHY module implementation
- `nv_phy_instance.hpp:34-112` - PHY instance base class
- `scf_5g_fapi_phy.hpp:65-270` - FAPI PHY interface
- `scf_5g_fapi_phy.cpp:141-5200` - FAPI PHY implementation

### PUSCH Processing
- `scf_5g_fapi_phy.cpp:1799-2069` - PUSCH validation (`on_pusch_pdu_info`)
- `scf_5g_fapi_phy.cpp:2072-2400` - Slot command prep (`prepare_ul_slot_command`)
- `scf_5g_fapi_phy.cpp:4722-4850` - CRC indication (`send_crc_indication`)
- `scf_5g_fapi_phy.cpp:4868-5000` - RX_DATA indication (`send_rx_data_indication`)
- `scf_5g_fapi_phy.cpp:4143-4550` - UCI indication (`send_uci_indication`)

### Data Structures
- `scf_5g_slot_commands_common.hpp:1-435` - Slot command structures
- `scf_5g_fapi.h` - FAPI message definitions
- `nv_phy_limit_errors.hpp:1-29` - L1 limit error structures

### Thread Management
- `nv_phy_module.cpp:495-536` - Thread initialization
- `nv_phy_module.cpp:764-849` - Auxiliary threads
- `nv_phy_driver_proxy.hpp` - PHY driver thread pool

### Validation
- `scf_5g_fapi_ul_validate.hpp` - UL validation interface
- `scf_5g_fapi_ul_validate.cpp` - UL validation implementation

---

*Document Version: 1.0*
*Generated: 2025-01-19*
*NVIDIA Aerial cuBB - cuPHY Control Plane*
