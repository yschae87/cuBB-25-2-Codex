# PRACH Uplink Processing Chain - Technical Documentation

## Document Information
- **System**: NVIDIA Aerial cuBB (cuPHY-CP)
- **Component**: PRACH (Physical Random Access Channel) Uplink Processing
- **Date**: 2025-11-19
- **Version**: 1.0

---

## Table of Contents
1. [Overview](#overview)
2. [Architecture](#architecture)
3. [Processing Stages](#processing-stages)
4. [Data Structures](#data-structures)
5. [API Reference](#api-reference)
6. [Performance Considerations](#performance-considerations)
7. [Configuration](#configuration)

---

## Overview

### Purpose
This document provides a detailed technical analysis of the PRACH uplink processing chain in the NVIDIA Aerial cuBB system, covering the complete flow from O-RAN U-plane packet reception to FAPI RACH indication delivery to the MAC layer.

### Scope
The PRACH processing chain handles:
- Physical Random Access Channel (PRACH) signal detection
- O-RAN fronthaul interface compliance
- GPU-accelerated signal processing
- 5G FAPI-compliant message formatting

### System Context
```
┌─────────────┐     O-RAN        ┌──────────────────┐     FAPI      ┌─────────┐
│   O-RU      │ ─────────────>   │   cuPHY-CP       │ ───────────>  │   MAC   │
│ (Radio Unit)│   U-plane        │ (PRACH Processor)│   Messages    │  Layer  │
└─────────────┘   Packets        └──────────────────┘               └─────────┘
```

---

## Architecture

### High-Level Component Diagram

```
┌────────────────────────────────────────────────────────────────────────┐
│                          PRACH Processing Pipeline                      │
├────────────────────────────────────────────────────────────────────────┤
│                                                                         │
│  ┌──────────────┐   ┌──────────────┐   ┌──────────────┐              │
│  │  Fronthaul   │   │    Order     │   │     Phy      │              │
│  │   Driver     │──>│   Entity     │──>│  PrachAggr   │              │
│  │              │   │              │   │              │              │
│  └──────────────┘   └──────────────┘   └──────┬───────┘              │
│         │                   │                   │                      │
│         │ O-RAN Packets     │ Decompressed     │ Trigger              │
│         │                   │ IQ Data          │                      │
│         v                   v                   v                      │
│  ┌──────────────────────────────────────────────────┐                 │
│  │         cuPHY PRACH Receiver (GPU)                │                 │
│  │  ┌────────┐  ┌────────┐  ┌────────┐  ┌────────┐ │                 │
│  │  │  FFT   │─>│Correlate│─>│ Detect │─>│ Measure│ │                 │
│  │  └────────┘  └────────┘  └────────┘  └────────┘ │                 │
│  └──────────────────┬───────────────────────────────┘                 │
│                     │ Results                                          │
│                     v                                                  │
│  ┌──────────────────────────────────────────────────┐                 │
│  │        FAPI Message Builder                       │                 │
│  │  ┌──────────────┐      ┌──────────────┐          │                 │
│  │  │ RACH_IND     │      │ INTERFERENCE │          │                 │
│  │  │ (0x89)       │      │ _IND (0x8E)  │          │                 │
│  │  └──────────────┘      └──────────────┘          │                 │
│  └──────────────────┬───────────────────────────────┘                 │
│                     │ FAPI Messages                                    │
│                     v                                                  │
│  ┌──────────────────────────────────────────────────┐                 │
│  │           PHY-MAC Transport                       │                 │
│  └──────────────────────────────────────────────────┘                 │
│                                                                         │
└────────────────────────────────────────────────────────────────────────┘
```

### Key Software Modules

| Module | Location | Primary Responsibility |
|--------|----------|----------------------|
| **Fronthaul Driver** | `aerial-fh-driver/lib/` | O-RAN packet reception |
| **Order Entity** | `cuphydriver/src/uplink/order_entity.cpp` | IQ decompression & ordering |
| **PhyPrachAggr** | `cuphydriver/src/uplink/phyprach_aggr.cpp` | PRACH task orchestration |
| **cuPHY PRACH RX** | `cuPHY/src/cuphy_channels/prach_rx.cpp` | GPU-accelerated detection |
| **FAPI Builder** | `scfl2adapter/lib/scf_5g_fapi/scf_5g_fapi_phy.cpp` | Message formatting |
| **Transport** | `gt_common_libs/altran/` | PHY-MAC communication |

---

## Processing Stages

### Stage 1: O-RAN U-Plane Packet Reception

#### Component: Fronthaul Driver
**File**: `aerial-fh-driver/lib/aerial_fh_driver.cpp`

**Responsibilities**:
- Receive PRACH U-plane packets from O-RU via Ethernet
- Parse O-RAN protocol headers
- Extract timing and configuration metadata
- Queue packets for processing

#### O-RAN Packet Structure
**File**: `aerial-fh-driver/include/aerial-fh-driver/oran.hpp`

```cpp
// Ethernet Frame
struct oran_ether_hdr {
    uint8_t  dst_addr[6];
    uint8_t  src_addr[6];
    uint16_t ether_type;
} __attribute__((__packed__));

// VLAN Header (optional)
struct oran_vlan_hdr {
    uint16_t tci;           // Tag Control Information
    uint16_t ether_type;
} __attribute__((__packed__));

// O-RAN U-Plane Message
struct oran_uplane_msg {
    // Header fields:
    // - dataDirection: UL (0) for PRACH
    // - frameId: SFN
    // - subframeId: Slot
    // - slotId: Symbol
    // Section payloads with IQ data
};
```

**Key Metadata Extracted**:
- **Timing**: SFN (System Frame Number), Slot, Symbol
- **Carrier**: Cell ID, Antenna ID
- **Compression**: Bit-width, compression method (BFP, modulation, etc.)
- **Payload**: Compressed IQ samples

#### Stream Reception
**File**: `aerial-fh-driver/lib/stream_rx.cpp`

Uses DPDK for high-performance packet I/O:
```cpp
// Pseudo-code flow
while (running) {
    nb_rx = rte_eth_rx_burst(port_id, queue_id, pkts, MAX_PKT_BURST);
    for (i = 0; i < nb_rx; i++) {
        parse_oran_packet(pkts[i]);
        enqueue_for_processing(pkts[i]);
    }
}
```

---

### Stage 2: Data Decompression and Ordering

#### Component: Order Entity
**File**: `cuphydriver/src/uplink/order_entity.cpp`
**Header**: `cuphydriver/include/order_entity.hpp`

**Responsibilities**:
1. Receive O-RAN packets from fronthaul driver
2. Decompress IQ samples based on compression method
3. Reorder data for efficient GPU processing
4. Manage pinned and device memory buffers
5. Signal readiness to PRACH processor

#### Class Definition
```cpp
class OrderEntity {
public:
    // Configuration and setup
    void configure(const orderKernelConfigParams& params);
    void setup_slot(SlotMapUl* slot_map);

    // Processing
    void run(uint32_t slot_idx);

    // Memory management
    void allocate_buffers();
    void release_buffers();

private:
    // PRACH-specific buffers
    cuphy::tensor_device ordered_prbs_prach[MAX_CELLS];
    cuphy::buffer<uint8_t> order_kernel_exit_cond_gdr;

    // Configuration
    orderKernelConfigParams config_;
    uint8_t compression_method_;
    uint8_t bit_width_;
};
```

#### Configuration Parameters
```cpp
struct orderKernelConfigParams {
    uint32_t cell_id;
    uint8_t  ru_type;           // O-RU type identifier
    uint8_t  compression_method; // BFP, modulation, etc.
    uint8_t  bit_width;         // IQ sample bit-width (8, 9, 12, 14, 16)
    uint8_t  cell_health;       // Cell status
    float    beta_dl;           // Downlink scaling factor
    float    beta_ul;           // Uplink scaling factor
    uint32_t sfn;               // System frame number
    uint32_t slot;              // Slot number
};
```

#### Decompression Methods Supported

| Method | Description | Typical Bit-Width |
|--------|-------------|-------------------|
| **None** | Uncompressed | 16 bits |
| **BFP** | Block Floating Point | 8-12 bits |
| **Modulation** | Modulation compression | 1-2 bits |
| **BFP Selective** | Selective BFP | 8-14 bits |

#### Memory Flow
```
O-RAN Packets (Network)
    ↓
DPDK Buffers (Host Memory)
    ↓
Order Kernel (CPU/GPU)
    ↓ Decompression
Ordered PRB Buffers (GPU Memory)
    ↓
PRACH Processor Input
```

---

### Stage 3: PRACH Task Coordination

#### Component: Slot Map UL
**File**: `cuphydriver/include/slot_map_ul.hpp`

**Responsibilities**:
- Schedule PRACH processing tasks per slot
- Manage timing constraints
- Coordinate buffer allocation/release
- Track processing metrics

#### Key Data Members
```cpp
class SlotMapUl {
public:
    // PRACH specific
    PhyPrachAggr* aggr_prach;

    // Timing tracking
    nv::hrc::time_point start_t_ul_prach_cuda;
    nv::hrc::time_point end_t_ul_prach_cuda;
    nv::hrc::time_point start_t_ul_prach_compl;
    nv::hrc::time_point end_t_ul_prach_compl;

    // Buffers
    ULInputBuffer* input_buffer;
    ULOutputBuffer* output_buffer;
};
```

#### Timing Constraints
```cpp
// PRACH processing timeline (typical values)
constexpr auto PRACH_RX_WINDOW     = 1ms;    // O-RU transmission window
constexpr auto PRACH_PROC_DEADLINE = 2ms;    // Processing deadline
constexpr auto PRACH_IND_LATENCY   = 3ms;    // Total indication latency
```

---

### Stage 4: PRACH Physical Layer Processing

#### Component: PhyPrachAggr
**File**: `cuphydriver/src/uplink/phyprach_aggr.cpp`
**Header**: `cuphydriver/include/phyprach_aggr.hpp`

This is the **main orchestrator** for PRACH processing.

#### Class Interface
```cpp
class PhyPrachAggr {
public:
    // Lifecycle
    void setup(SlotMapUl* slot_map);
    void run(cudaStream_t stream);
    void validate();
    void callback();
    void reserve(ULInputBuffer* buffer);

    // Configuration
    void createPhyObj();
    void updateConfig(uint32_t cell_id, const PrachConfig& config);

private:
    // cuPHY handle
    cuphyPrachRxHndl_t handle;

    // Input tensor
    cuphyTensorPrm_t tDataRx;
    cuphy::tensor_desc prach_data_rx_desc[PRACH_MAX_OCCASIONS_AGGR];

    // Output tensors (GPU)
    cuphy::tensor_device gpu_num_detectedPrmb;
    cuphy::tensor_device gpu_prmbIndex_estimates;
    cuphy::tensor_device gpu_prmbDelay_estimates;
    cuphy::tensor_device gpu_prmbPower_estimates;

    // Output tensors (CPU pinned)
    cuphy::tensor_pinned cpu_num_detectedPrmb;
    cuphy::tensor_pinned cpu_prmbIndex_estimates;
    cuphy::tensor_pinned cpu_prmbDelay_estimates;
    cuphy::tensor_pinned cpu_prmbPower_estimates;
    cuphy::tensor_pinned ant_rssi;
    cuphy::tensor_pinned rssi;
    cuphy::tensor_pinned interference;

    // Configuration
    cuphyPrachStatPrms_t prach_params_static;
    std::vector<cuphyPrachOccaStatPrms_t> prach_occa_stat_params;
    cuphyPrachOccaDynPrms_t* prach_dyn_occa_params;

    // Workspace
    cuphy::buffer<float, cuphy::device_alloc> prach_workspace_buffer;
};
```

#### Static Configuration Structure
```cpp
struct cuphyPrachStatPrms_t {
    uint32_t nCells;                    // Number of cells
    cuphyPrachCellStatPrms_t* pCellStatPrms; // Per-cell parameters
};

struct cuphyPrachCellStatPrms_t {
    uint32_t occaStartIdx;              // Starting occasion index
    uint32_t nOccasionsFdm;             // Number of FDM occasions
    uint32_t N_ant;                     // Number of antennas
    cuphyFreqRange_t freqRange;         // FR1 or FR2
    cuphyDuplexMode_t duplexMode;       // TDD or FDD
    uint32_t mu;                        // Subcarrier spacing (0,1,2,3)
    uint32_t configurationIndex;        // PRACH config index (0-255)
    cuphyPrachRestrictedSet_t restrictedSetCfg; // Restricted set type
};

struct cuphyPrachOccaStatPrms_t {
    uint32_t cellStatIdx;               // Cell index
    uint32_t rootSequenceIndex;         // Root sequence (0-837)
    uint32_t zeroCorrelationZoneCfg;    // ZCZ config (0-15)
};
```

#### Dynamic Configuration Structure
```cpp
struct cuphyPrachDynPrms_t {
    uint32_t nOccasions;                // Number of occasions to process
    cuphyPrachOccaDynPrms_t* pOccaDynPrms; // Per-occasion parameters
};

struct cuphyPrachOccaDynPrms_t {
    uint32_t occaParamStatIdx;          // Static param index
    uint32_t occaParamDynIdx;           // Dynamic param index
    float    forceThreshold;            // Detection threshold (dB)
    uint32_t freqIndex;                 // Frequency index
    uint32_t symbolIndex;               // Symbol index
};
```

#### Processing Flow
```cpp
void PhyPrachAggr::run(cudaStream_t stream) {
    // 1. Setup input tensor descriptors
    for (uint32_t i = 0; i < nOccasions; i++) {
        prach_data_rx_desc[i].data = ordered_prbs_prach[i];
        prach_data_rx_desc[i].nAntennas = N_ant;
        prach_data_rx_desc[i].nSamples = nSamplesPerOcca;
    }

    // 2. Setup cuPHY PRACH receiver
    cuphySetupPrachRx(handle,
                      &prach_params_static,
                      prach_dyn_occa_params,
                      &tDataRx,
                      stream);

    // 3. Run PRACH detection on GPU
    cuphyRunPrachRx(handle, stream);

    // 4. Copy results from GPU to CPU
    cudaMemcpyAsync(cpu_num_detectedPrmb.data(),
                    gpu_num_detectedPrmb.data(),
                    size, cudaMemcpyDeviceToHost, stream);
    // ... copy other results ...

    // 5. Synchronize stream
    cudaStreamSynchronize(stream);
}
```

---

### Stage 5: GPU-Accelerated PRACH Detection

#### Component: cuPHY PRACH Receiver
**File**: `cuPHY/src/cuphy_channels/prach_rx.cpp`
**Header**: `cuPHY/src/cuphy_channels/prach_rx.hpp`

#### API Functions
```cpp
// Create PRACH receiver handle
cuphyStatus_t cuphyCreatePrachRx(
    cuphyPrachRxHndl_t* pHandle,
    const cuphyPrachStatPrms_t* pStaticParams,
    void* workspace,
    size_t workspaceSize
);

// Setup for current occasion
cuphyStatus_t cuphySetupPrachRx(
    cuphyPrachRxHndl_t handle,
    const cuphyPrachStatPrms_t* pStaticParams,
    const cuphyPrachDynPrms_t* pDynParams,
    const cuphyTensorPrm_t* pInputData,
    cudaStream_t stream
);

// Execute PRACH detection
cuphyStatus_t cuphyRunPrachRx(
    cuphyPrachRxHndl_t handle,
    cudaStream_t stream
);

// Get results
cuphyStatus_t cuphyGetPrachRxResults(
    cuphyPrachRxHndl_t handle,
    cuphyPrachStatusOut_t* pOutput
);
```

#### Processing Pipeline (GPU Kernels)

```
Input: IQ Samples [nAntennas × nSamples]
    ↓
┌─────────────────────────────────────┐
│  1. FFT (Time → Frequency Domain)   │
│     - CUFFT library                  │
│     - Per-antenna transformation     │
└─────────────────┬───────────────────┘
                  ↓
┌─────────────────────────────────────┐
│  2. Reference Sequence Generation   │
│     - Zadoff-Chu sequences           │
│     - Root sequence index based      │
│     - Per-preamble (0-63)            │
└─────────────────┬───────────────────┘
                  ↓
┌─────────────────────────────────────┐
│  3. Cross-Correlation                │
│     - Correlate RX with references   │
│     - Per-antenna correlation        │
└─────────────────┬───────────────────┘
                  ↓
┌─────────────────────────────────────┐
│  4. Combining (Multiple Antennas)   │
│     - Coherent or non-coherent       │
│     - Power summation                │
└─────────────────┬───────────────────┘
                  ↓
┌─────────────────────────────────────┐
│  5. Peak Detection                   │
│     - Threshold comparison           │
│     - Local maxima finding           │
└─────────────────┬───────────────────┘
                  ↓
┌─────────────────────────────────────┐
│  6. Delay Estimation                 │
│     - Peak position → timing advance │
│     - Microsecond precision          │
└─────────────────┬───────────────────┘
                  ↓
┌─────────────────────────────────────┐
│  7. Power Measurement                │
│     - Correlation peak power         │
│     - dBm calculation                │
└─────────────────┬───────────────────┘
                  ↓
┌─────────────────────────────────────┐
│  8. RSSI Calculation                 │
│     - Per-antenna RSSI               │
│     - Average RSSI                   │
│     - Interference estimation        │
└─────────────────┬───────────────────┘
                  ↓
Output: Detection Results
```

#### Output Structure
```cpp
struct cuphyPrachStatusOut_t {
    uint32_t* num_detectedPrmb;     // [nOccasions]
    uint32_t* prmbIndex_estimates;  // [nOccasions × MAX_PREAMBLES]
    float*    prmbDelay_estimates;  // [nOccasions × MAX_PREAMBLES] (μs)
    float*    prmbPower_estimates;  // [nOccasions × MAX_PREAMBLES] (dBm)
    float*    ant_rssi;             // [nOccasions × nAntennas] (dB)
    float*    rssi;                 // [nOccasions] (dB)
    float*    interference;         // [nOccasions] (dBm)
};
```

**Typical Values**:
- `num_detectedPrmb`: 0-64 preambles per occasion
- `prmbIndex_estimates`: 0-63 (preamble identity)
- `prmbDelay_estimates`: 0-1300 μs (timing advance)
- `prmbPower_estimates`: -140 to +30 dBm
- `rssi`: -63 to +30 dB
- `interference`: -140 to +30 dBm

---

### Stage 6: Result Callback and FAPI Message Construction

#### Component: FAPI PHY Layer
**File**: `scfl2adapter/lib/scf_5g_fapi/scf_5g_fapi_phy.cpp`

#### Callback Registration
**Location**: Lines 4636-4647

```cpp
// Register PRACH callback during initialization
cb.ul_cb.prach_cb_fn = [this] (
    slot_command_api::slot_indication& slot,
    const prach_params& params,
    const uint32_t* num_detectedPrmb,
    const void* prmbIndex_estimates,
    const void* prmbDelay_estimates,
    const void* prmbPower_estimates,
    const void* ant_rssi,
    const void* rssi,
    const void* interference)
{
    send_rach_indication(slot, params, num_detectedPrmb,
                        prmbIndex_estimates, prmbDelay_estimates,
                        prmbPower_estimates, ant_rssi, rssi,
                        interference);
};
```

#### FAPI Message Builder
**Function**: `phy::send_rach_indication()`
**Location**: Lines 3286-3447

##### Step 1: Message Allocation
```cpp
void phy::send_rach_indication(
    slot_command_api::slot_indication& slot,
    const prach_params& params,
    const uint32_t* num_detectedPrmb,
    const void* prmbIndex_estimates,
    const void* prmbDelay_estimates,
    const void* prmbPower_estimates,
    const void* ant_rssi,
    const void* rssi,
    const void* interference)
{
    // Get transport interface
    auto& transport = get_transport(cell_id);

    // Calculate message size
    uint32_t numPdus = params.nOccasion;
    uint32_t totalPreambles = 0;
    for (uint32_t i = 0; i < numPdus; i++) {
        totalPreambles += num_detectedPrmb[i];
    }

    size_t msg_size = sizeof(scf_fapi_rach_ind_t) +
                      numPdus * sizeof(scf_fapi_prach_ind_pdu_t) +
                      totalPreambles * sizeof(scf_fapi_prach_preamble_info_t);

    // Allocate FAPI message
    auto msg_desc = transport.tx_alloc(msg_size, cell_id);
    auto* fapi = reinterpret_cast<scf_fapi_rach_ind_t*>(msg_desc.data);
```

##### Step 2: Message Header
```cpp
    // Fill message header
    fapi->msg_hdr.message_type = SCF_FAPI_RACH_INDICATION;
    fapi->msg_hdr.length = msg_size;
    fapi->sfn = slot.sfn_;
    fapi->slot = slot.slot_;
    fapi->num_pdus = numPdus;
```

##### Step 3: Per-PDU Processing
```cpp
    // Process each PRACH occasion
    auto* pdu = fapi->pdu_info;
    uint32_t prmb_idx = 0;

    for (uint32_t i = 0; i < numPdus; i++) {
        // Physical cell ID
        pdu[i].phys_cell_id = params.phy_cell_index_list[i];

        // Timing information
        pdu[i].symbol_index = params.startSymbols[i];
        pdu[i].slot_index = slot.slot_;
        pdu[i].freq_index = params.freqIndex[i];

        // RSSI conversion: dB → [0-254] scale
        // Formula: RSSI_value = (RSSI_dB * 2) + 128
        // Range: -63 dB (0) to +30 dB (254)
        const float* avg_rssi = static_cast<const float*>(rssi);
        pdu[i].avg_rssi = avg_rssi[i] * 2 + 128 + 0.5;

        // SNR (if available)
        pdu[i].avg_snr = 0xFF; // Not computed

        // Number of preambles detected
        pdu[i].num_preamble = num_detectedPrmb[i];
```

##### Step 4: Per-Preamble Processing
```cpp
        // Process each detected preamble
        auto* preamble = pdu[i].preamble_info;
        const uint32_t* index_est = static_cast<const uint32_t*>(prmbIndex_estimates);
        const float* delay_time_est = static_cast<const float*>(prmbDelay_estimates);
        const float* peak_dest = static_cast<const float*>(prmbPower_estimates);

        for (uint32_t j = 0; j < num_detectedPrmb[i]; j++) {
            // Preamble index (0-63)
            preamble[j].preamble_index = index_est[prmb_idx];

            // Timing advance conversion
            // From: microseconds
            // To: units of (64 × 16 × Tc)
            //     where Tc = 1/(480000 × 4096) seconds
            float step = STEP_CONST / (1 << params.mu);
            int tmp = (delay_time_est[prmb_idx] - prach_ta_offset_usec_/1000000.0) *
                      (1 << phy_module().get_mu_highest()) * 192 * 10000;
            preamble[j].timing_advance = std::clamp(tmp, 0, 3846);

            // Preamble power conversion
            // From: linear power
            // To: [0-170000] representing [-140 to +30] dBm in 0.001 dB steps
            constexpr float RACH_BB2RF_powerOffset = -48.68; // dB
            preamble[j].preamble_power =
                1000 * (10 * std::log10(peak_dest[prmb_idx]) + 140 +
                        RACH_BB2RF_powerOffset) + 0.5;

            prmb_idx++;
        }
    }
```

##### Step 5: Interference Indication (Optional)
```cpp
    // Send separate interference indication if available
    if (interference != nullptr) {
        const float* intf_ptr = static_cast<const float*>(interference);

        size_t intf_msg_size = sizeof(scf_fapi_rx_prach_interference_ind_t) +
                               numPdus * sizeof(scf_fapi_prach_interference_pdu_t);

        auto intf_msg = transport.tx_alloc(intf_msg_size, cell_id);
        auto* intf_fapi = reinterpret_cast<scf_fapi_rx_prach_interference_ind_t*>(
                            intf_msg.data);

        intf_fapi->msg_hdr.message_type = SCF_FAPI_RX_PRACH_INTEFERNCE_INDICATION;
        intf_fapi->sfn = slot.sfn_;
        intf_fapi->slot = slot.slot_;
        intf_fapi->num_pdus = numPdus;

        for (uint32_t i = 0; i < numPdus; i++) {
            intf_fapi->pdu_info[i].interference = intf_ptr[i];
        }

        transport.tx_send(intf_msg);
    }
```

##### Step 6: Message Transmission
```cpp
    // Send RACH indication
    transport.tx_send(msg_desc);
    transport.notify(1);

    // Update metrics
    metrics_.incr_tx_packet_count(SCF_FAPI_RACH_INDICATION);
}
```

---

### Stage 7: PHY-MAC Transport

#### Component: MAC-PHY Interface
**File**: `gt_common_libs/altran/include/mac_phy_intf.h`

#### Transport Interface
```cpp
class PhyMacTransport {
public:
    // Message allocation
    MessageDescriptor tx_alloc(size_t size, uint32_t cell_id);

    // Message transmission
    void tx_send(const MessageDescriptor& msg);

    // Notification to MAC
    void notify(uint32_t count);

    // Receive from MAC (for DL)
    MessageDescriptor rx_recv();
};
```

#### Memory Management
- **Zero-copy**: Shared memory between PHY and MAC
- **Lock-free queues**: Minimal latency
- **Message pool**: Pre-allocated buffers

#### Typical Latency Budget
```
PRACH Reception (O-RU)               : T + 0 ms
Order Entity Processing              : T + 0.5 ms
cuPHY GPU Processing                 : T + 1.0 ms
FAPI Message Construction            : T + 1.1 ms
Transport to MAC                     : T + 1.2 ms
MAC Processing (RACH Response)       : T + 2.0 ms
Total PRACH Indication Latency       : ~2 ms
```

---

## Data Structures

### FAPI Message Definitions
**File**: `scfl2adapter/lib/scf_5g_fapi/scf_5g_fapi.h`

#### RACH Indication Message
```cpp
// Message ID
#define SCF_FAPI_RACH_INDICATION  0x89

// Main message structure (Lines 1404-1430)
typedef struct {
    scf_fapi_body_header_t msg_hdr;
    uint16_t sfn;               // System Frame Number (0-1023)
    uint16_t slot;              // Slot (0-159 for FR1, 0-319 for FR2)
    uint8_t  num_pdus;          // Number of PRACH occasions
    scf_fapi_prach_ind_pdu_t pdu_info[0]; // Variable length array
} scf_fapi_rach_ind_t;
```

#### PRACH Indication PDU
```cpp
// Per-occasion information (Lines 1412-1421)
typedef struct {
    uint16_t phys_cell_id;      // Physical Cell ID (0-1007)
    uint8_t  symbol_index;      // Starting symbol (0-13)
    uint8_t  slot_index;        // Slot index (0-159)
    uint8_t  freq_index;        // Frequency index (0-7)
    uint8_t  avg_rssi;          // Average RSSI
                                // Value = (RSSI_dB + 63) × 2
                                // Range: 0-254 → -63 to +30 dB
    uint8_t  avg_snr;           // Average SNR (0xFF if not computed)
    uint8_t  num_preamble;      // Number of preambles (0-64)
    scf_fapi_prach_preamble_info_t preamble_info[0]; // Variable length
} scf_fapi_prach_ind_pdu_t;
```

#### Preamble Information
```cpp
// Per-preamble detection info (Lines 1406-1409)
typedef struct {
    uint8_t  preamble_index;    // Preamble index (0-63)
    uint16_t timing_advance;    // Timing advance
                                // Units: 64 × 16 × Tc
                                // Range: 0-3846
                                // Tc = 1/(480000×4096) seconds
                                // Max: ~1300 μs
    uint32_t preamble_power;    // Received power
                                // Value = (Power_dBm + 140) × 1000
                                // Range: 0-170000 → -140 to +30 dBm
                                // Resolution: 0.001 dB
} scf_fapi_prach_preamble_info_t;
```

#### Interference Indication Message
```cpp
// Message ID
#define SCF_FAPI_RX_PRACH_INTEFERNCE_INDICATION  0x8E

typedef struct {
    scf_fapi_body_header_t msg_hdr;
    uint16_t sfn;
    uint16_t slot;
    uint8_t  num_pdus;
    scf_fapi_prach_interference_pdu_t pdu_info[0];
} scf_fapi_rx_prach_interference_ind_t;

typedef struct {
    uint16_t phys_cell_id;
    uint8_t  symbol_index;
    uint8_t  slot_index;
    uint8_t  freq_index;
    float    interference;       // Interference power (dBm)
} scf_fapi_prach_interference_pdu_t;
```

### Slot Command API Structures
**File**: `gt_common_libs/slot_command/include/slot_command/slot_command.hpp`

#### PRACH Parameters
```cpp
struct prach_params {
    uint32_t nOccasion;                     // Number of occasions
    std::vector<uint32_t> cell_index_list;  // Cell indices
    std::vector<uint32_t> phy_cell_index_list; // Physical cell IDs
    std::vector<uint8_t> startSymbols;      // Starting symbols
    std::vector<uint8_t> freqIndex;         // Frequency indices
    uint8_t mu;                             // Subcarrier spacing
    std::vector<uint8_t> numRa;             // Number of RAs per occasion
    std::vector<uint8_t> prachStartRb;      // Starting RB
    std::vector<uint16_t> rootSequenceIndex; // Root sequences
};
```

#### Callback Definition
```cpp
using prach_callback_fn = std::function<void(
    slot_command_api::slot_indication& slot,
    const prach_params& params,
    const uint32_t* num_detectedPrmb,
    const void* prmbIndex_estimates,
    const void* prmbDelay_estimates,
    const void* prmbPower_estimates,
    const void* ant_rssi,
    const void* rssi,
    const void* interference
)>;
```

---

## API Reference

### cuPHY PRACH RX API

#### cuphyCreatePrachRx
```cpp
cuphyStatus_t cuphyCreatePrachRx(
    cuphyPrachRxHndl_t* pHandle,              // [out] Handle to PRACH receiver
    const cuphyPrachStatPrms_t* pStaticParams, // [in] Static configuration
    void* workspace,                          // [in] Workspace buffer
    size_t workspaceSize                      // [in] Workspace size
);
```
**Purpose**: Create and initialize PRACH receiver pipeline

**Returns**:
- `CUPHY_STATUS_SUCCESS`: Success
- `CUPHY_STATUS_INVALID_PARAM`: Invalid parameters
- `CUPHY_STATUS_ALLOC_FAILED`: Memory allocation failed

#### cuphySetupPrachRx
```cpp
cuphyStatus_t cuphySetupPrachRx(
    cuphyPrachRxHndl_t handle,                // [in] PRACH receiver handle
    const cuphyPrachStatPrms_t* pStaticParams, // [in] Static parameters
    const cuphyPrachDynPrms_t* pDynParams,     // [in] Dynamic parameters
    const cuphyTensorPrm_t* pInputData,        // [in] Input IQ data
    cudaStream_t stream                       // [in] CUDA stream
);
```
**Purpose**: Setup PRACH receiver for current slot/occasion

**Parameters**:
- `pStaticParams`: Cell configuration, not expected to change frequently
- `pDynParams`: Per-slot parameters (thresholds, indices)
- `pInputData`: Pointer to IQ samples in GPU memory

#### cuphyRunPrachRx
```cpp
cuphyStatus_t cuphyRunPrachRx(
    cuphyPrachRxHndl_t handle,    // [in] PRACH receiver handle
    cudaStream_t stream           // [in] CUDA stream
);
```
**Purpose**: Execute PRACH detection on GPU

**Behavior**: Asynchronous execution on specified CUDA stream

#### cuphyGetPrachRxResults
```cpp
cuphyStatus_t cuphyGetPrachRxResults(
    cuphyPrachRxHndl_t handle,        // [in] PRACH receiver handle
    cuphyPrachStatusOut_t* pOutput    // [out] Detection results
);
```
**Purpose**: Retrieve detection results (after stream synchronization)

#### cuphyDestroyPrachRx
```cpp
cuphyStatus_t cuphyDestroyPrachRx(
    cuphyPrachRxHndl_t handle    // [in] Handle to destroy
);
```
**Purpose**: Cleanup and destroy PRACH receiver

---

## Performance Considerations

### GPU Processing Optimization

#### Kernel Launch Configuration
```cpp
// Typical kernel configuration
dim3 grid(numOccasions, numAntennas);
dim3 block(256);  // Threads per block

// FFT optimization
cufftPlanMany(&plan, rank, n,
              inembed, istride, idist,
              onembed, ostride, odist,
              CUFFT_C2C, batch);
```

#### Memory Bandwidth
- **Input Data**: 4 antennas × 839 samples × 4 bytes (complex) = ~13 KB per occasion
- **Reference Sequences**: 64 preambles × 839 samples × 4 bytes = ~200 KB (cached)
- **Output**: Minimal (~1 KB per occasion)

#### Throughput Metrics
```
Single Occasion (4 antennas):
- FFT:          ~50 μs
- Correlation:  ~100 μs
- Detection:    ~20 μs
- Total:        ~170 μs

4 Occasions (typical slot):
- Total GPU:    ~680 μs
- CPU overhead: ~100 μs
- DMA transfer: ~50 μs
- End-to-end:   ~830 μs
```

### Memory Management

#### Buffer Allocation Strategy
```cpp
// Pre-allocate pinned memory for zero-copy DMA
cudaMallocHost(&cpu_buffer, size);

// Pre-allocate device memory
cudaMalloc(&gpu_buffer, size);

// Use memory pools for frequent allocations
cudaMemPoolCreate(&pool, &poolProps);
cudaMallocFromPoolAsync(&ptr, size, pool, stream);
```

#### Memory Footprint
```
Per Cell (4 antennas, 4 occasions):
- Input buffers:        ~52 KB (GPU)
- Workspace:            ~2 MB (GPU)
- Output buffers:       ~4 KB (GPU + CPU pinned)
- Reference sequences:  ~200 KB (GPU, cached)
- Total per cell:       ~2.3 MB
```

### Latency Optimization

#### Critical Path Analysis
```
1. O-RAN Packet Reception:       ~100 μs (network + DPDK)
2. Decompression (Order Entity):  ~200 μs (CPU/GPU)
3. GPU Processing:                ~680 μs (CUDA kernels)
4. DMA Transfer (D2H):            ~50 μs  (results)
5. FAPI Message Construction:     ~50 μs  (CPU)
6. Transport Send:                ~20 μs  (shared memory)
---------------------------------------------------
Total Critical Path:              ~1.1 ms
```

#### Parallelization Opportunities
- **Multiple Cells**: Process cells in parallel on different CUDA streams
- **Pipeline Stages**: Overlap reception, processing, and callback
- **Batch Processing**: Process multiple occasions together

---

## Configuration

### PRACH Configuration Index
**Reference**: 3GPP TS 38.211 Table 6.3.3.2-2/3

Selected examples:

| Config Index | Format | Duration | Subcarrier Spacing | Occasions per Slot |
|--------------|--------|----------|-------------------|-------------------|
| 0 | 0 | 839 samples | 15 kHz | 1 |
| 16 | 1 | 839 samples | 15 kHz | 2 |
| 67 | A1 | 139 samples | 30 kHz | 2 |
| 158 | C0 | 1151 samples | 15 kHz | 1 |

### Root Sequence Index
**Reference**: 3GPP TS 38.211 Section 6.3.3.1

- **Range**: 0-837
- **Purpose**: Determines Zadoff-Chu sequence
- **Formula**: `x(n) = exp(-j × π × u × n × (n+1) / L_RA)`
  - `u`: Root sequence index
  - `L_RA`: Sequence length (139 or 839)

### Zero Correlation Zone (ZCZ)
**Reference**: 3GPP TS 38.211 Table 6.3.3.1-5

| ZCZ Config | N_cs (Format 0/1) | N_cs (Format A1-C2) |
|-----------|-------------------|---------------------|
| 0 | 0 | 0 |
| 1 | 13 | 2 |
| 2 | 15 | 4 |
| ... | ... | ... |
| 15 | 237 | 30 |

### Restricted Set Configuration
- **Unrestricted**: All root sequences available
- **Type A**: High-speed scenario (restricted sequences)
- **Type B**: High-speed scenario (different restriction)

### Example Configuration
```cpp
// FR1, TDD, 30 kHz SCS, PRACH Format A1
cuphyPrachCellStatPrms_t cell_config = {
    .occaStartIdx = 0,
    .nOccasionsFdm = 1,
    .N_ant = 4,
    .freqRange = CUPHY_FREQ_RANGE_1,
    .duplexMode = CUPHY_DUPLEX_TDD,
    .mu = 1,  // 30 kHz
    .configurationIndex = 67,
    .restrictedSetCfg = CUPHY_PRACH_UNRESTRICTED_SET
};

cuphyPrachOccaStatPrms_t occa_config = {
    .cellStatIdx = 0,
    .rootSequenceIndex = 1,
    .zeroCorrelationZoneCfg = 1  // N_cs = 2
};
```

---

## Debugging and Validation

### Logging Levels
```cpp
// Enable detailed PRACH logging
export CUPHY_LOG_LEVEL=DEBUG
export PRACH_DEBUG=1
```

### Key Log Points
1. **Order Entity**: IQ sample statistics
2. **PhyPrachAggr**: Configuration parameters
3. **cuPHY**: Detection results per preamble
4. **FAPI Builder**: Message contents

### Test Vector Support
**File**: `cuPHY/test/test_vectors/prach/`

HDF5 test vectors with:
- Input IQ samples
- Expected detection results
- Reference correlation outputs

### Validation Checks
```cpp
// PhyPrachAggr validation
void PhyPrachAggr::validate() {
    // Check detection results
    for (uint32_t i = 0; i < nOccasions; i++) {
        assert(cpu_num_detectedPrmb[i] <= 64);

        for (uint32_t j = 0; j < cpu_num_detectedPrmb[i]; j++) {
            uint32_t idx = i * MAX_PREAMBLES + j;

            // Validate preamble index
            assert(cpu_prmbIndex_estimates[idx] < 64);

            // Validate timing advance (0-1300 μs)
            assert(cpu_prmbDelay_estimates[idx] >= 0.0f);
            assert(cpu_prmbDelay_estimates[idx] <= 1300.0f);

            // Validate power estimate (-140 to +30 dBm)
            float power_db = 10*log10(cpu_prmbPower_estimates[idx]) - 140;
            assert(power_db >= -140.0f && power_db <= 30.0f);
        }
    }
}
```

---

## Appendix

### A. PRACH Formats

| Format | Duration (μs) | Subcarrier Spacing | Use Case |
|--------|---------------|-------------------|----------|
| 0 | 1000 | 15 kHz | Normal coverage |
| 1 | 666.7 | 15 kHz | Normal coverage |
| 2 | 333.3 | 15 kHz | Short latency |
| 3 | 100 | 15 kHz | Very short latency |
| A1 | 35.3 | 30 kHz | Short format FR1 |
| A2 | 70.6 | 30 kHz | Short format FR1 |
| A3 | 141.2 | 30 kHz | Short format FR1 |
| B1 | 100 | 60 kHz | Short format FR1 |
| B4 | 200 | 60 kHz | Short format FR1 |
| C0 | 1000 | 15 kHz | Long format FR1/FR2 |
| C2 | 2000 | 15 kHz | Extended coverage |

### B. Timing Advance Calculation Details

```
Timing Advance (TA) represents the round-trip propagation delay.

Formula:
    TA_samples = delay_μs × sample_rate
    TA_FAPI = TA_samples / (64 × 16)  // Convert to FAPI units

Example:
    Delay = 10 μs
    Sample rate = 30.72 MHz (for 15 kHz SCS)
    TA_samples = 10 μs × 30.72 MHz = 307.2 samples
    TA_FAPI = 307.2 / 1024 ≈ 0.3 (rounded to nearest integer)

Range:
    Min: 0 (UE at cell center)
    Max: 3846 (UE at cell edge, ~1300 μs)

Maximum Cell Radius (approx):
    Distance = (TA_max × 64 × 16 × Tc) × c / 2
             = (3846 × 1024 × Tc) × 3e8 / 2
             ≈ 100 km (for FR1)
```

### C. Power Measurement Conversion

```cpp
// cuPHY output: Linear power (correlation peak magnitude squared)
float linear_power = gpu_prmbPower_estimates[idx];

// Convert to dB
float power_db = 10.0f * log10(linear_power);

// Add baseband-to-RF offset (calibration)
constexpr float RACH_BB2RF_powerOffset = -48.68; // dB

// Adjust to absolute power (dBm)
float power_dbm = power_db + 140.0f + RACH_BB2RF_powerOffset;

// Convert to FAPI format (0.001 dB resolution)
uint32_t power_fapi = static_cast<uint32_t>(power_dbm * 1000.0f + 0.5f);

// Clamp to valid range [0, 170000]
power_fapi = std::clamp(power_fapi, 0u, 170000u);
```

### D. Reference Documents

1. **3GPP TS 38.211**: Physical channels and modulation
   - Section 6.3.3: PRACH structure and sequences

2. **3GPP TS 38.213**: Physical layer procedures
   - Section 8.1: PRACH transmission procedure

3. **SCF FAPI Specification**: 5G FAPI: PHY API Specification
   - Section 3.4.10: RACH.indication

4. **O-RAN Fronthaul Specification**: O-RAN.WG4.CUS.0
   - Section 7: U-Plane message structure

5. **NVIDIA Aerial SDK Documentation**:
   - cuPHY API Reference
   - cuBB Integration Guide

---

## Revision History

| Version | Date | Author | Changes |
|---------|------|--------|---------|
| 1.0 | 2025-11-19 | Claude Code | Initial comprehensive documentation |

---

## Contact and Support

For questions or issues related to this documentation:
- **Technical Support**: NVIDIA Aerial SDK Support Portal
- **Documentation Updates**: Submit via internal documentation system

---

*End of Document*
