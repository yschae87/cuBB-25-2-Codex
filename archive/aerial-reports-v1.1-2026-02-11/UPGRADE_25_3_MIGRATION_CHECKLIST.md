# Aerial 25.2 -> 25.3 Migration Checklist (v1.1)

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

## Snapshot
- Local workspace: `/Users/yschae87/Library/Mobile Documents/com~apple~CloudDocs/Nvidia/Aerial/cuBB-25-2-Codex`
- Local version marker: `25-2-cubb`
- Upstream source: `NVIDIA/aerial-cuda-accelerated-ran`
- Upstream tag used: `25.3.2`
- Upstream commit used: `3bf76a43dceb493b00f2ee75fdfbb87038eab7c6`
- Upstream version marker: `25-3-cubb`

## Safe Upgrade Workspace
- Created isolated 25.3 baseline copy at:
  - `/Users/yschae87/Library/Mobile Documents/com~apple~CloudDocs/Nvidia/Aerial/cuBB-25-2-Codex/upgrade-25-3-workspace`
- Source of that copy: `/tmp/aerial-upstream` checked out at tag `25.3.2`

## Drift Size (changed paths, local vs 25.3 baseline)
- `cmake`: `265`
- `cuPHY`: `2950`
- `cuPHY-CP`: `39658`
- `cuMAC`: `2085`
- `cuMAC-CP`: `28`
- `pyaerial`: `424`
- `CMakeLists.txt`: `1`

## Top-Level Layout Delta
### Local-only top-level entries
- `AGENTS.md`
- `CUPHYCONTROLLER_ANALYSIS.md`
- `CUPHYCONTROLLER_ROLES.md`
- `CUPHYDRIVER_ANALYSIS.md`
- `CUPHYDRIVER_ROLES.md`
- `LICENSE.txt`
- `MAC_ARCHITECTURE.md`
- `MULTITHREAD_PARALLEL_ARCHITECTURE-claude.md`
- `PDSCH_PIPELINE.md`
- `PRACH_PROCESSING_CHAIN.md`
- `UPLINK_MULTITHREAD_ARCHITECTURE.md`
- `UPLINK_PUSCH_PROCESSING_ARCHITECTURE.md`
- `architecture_analysis.md`
- `cuMAC-CP 2`
- `multi_task_parallel_architecture-v0.md`
- `multi_task_parallel_architecture.md`
- `pdsch_iq_bfp_report.md`
- `pdsch_precoding_analysis.md`
- `pdsch_processing_report.md`
- `upgrade-25-3-workspace`

### Upstream-only top-level entries
- `ATTRIBUTION.rst`
- `CMakePresets.json`
- `CONTRIBUTING.md`
- `LICENSE`
- `README.md`
- `SECURITY.md`

## Priority Migration Tracks

### Track 1 (highest): `cuPHY-CP`
Rationale: largest drift and most FH/compression/control-plane coupling.

Initial hotspots seen in diff sample:
- `cuPHY-CP/aerial-fh-driver/CMakeLists.txt`
- `cuPHY-CP/aerial-fh-driver/app/fh_generator/src/cell.cpp`
- `cuPHY-CP/aerial-fh-driver/app/fh_generator/src/cuda_kernels.cu`
- `cuPHY-CP/aerial-fh-driver/app/fh_generator/src/dl_tx_worker.cpp`
- `cuPHY-CP/aerial-fh-driver/app/fh_generator/src/fh_generator.cpp`

### Track 2: `cuPHY`
Rationale: PDSCH/DMRS/precoding/compression interactions.

Initial hotspots seen in diff sample:
- `cuPHY/src/cuphy/pdsch_dmrs/pdsch_dmrs.cu`
- `cuPHY/src/cuphy/pdsch_dmrs/pdsch_dmrs.hpp`
- `cuPHY/src/cuphy_channels/pdsch_tx.cpp`
- `cuPHY/src/cuphy_channels/pdsch_tx.hpp`
- `cuPHY/src/compression_decompression/bfw_blockFP.cuh`
- `cuPHY/src/compression_decompression/bfw_packing.cuh`

### Track 3: `cuMAC` then `pyaerial`
Rationale: API surface and wrappers should align after low-level core settles.

Initial `pyaerial` hotspots seen in diff sample:
- `pyaerial/pybind11/pycuphy_pdsch.cpp`
- `pyaerial/pybind11/pycuphy_pdsch.hpp`
- `pyaerial/pybind11/pycuphy_pdsch_params.cpp`
- `pyaerial/src/aerial/phy5g/pdsch/pdsch_tx.py`
- `pyaerial/src/aerial/phy5g/pdsch/separable_pdsch_tx.py`

## Exact Commands To Drive Migration

### 1) Recompute module drift quickly
```bash
LOCAL="/Users/yschae87/Library/Mobile Documents/com~apple~CloudDocs/Nvidia/Aerial/cuBB-25-2-Codex"
UP="$LOCAL/upgrade-25-3-workspace"
for d in cmake cuPHY cuPHY-CP cuMAC cuMAC-CP pyaerial; do
  c=$(git diff --no-index --name-only "$LOCAL/$d" "$UP/$d" 2>/dev/null | wc -l | tr -d ' ')
  printf "%s %s\n" "$d" "$c"
done
```

### 2) Generate file list for one module
```bash
LOCAL="/Users/yschae87/Library/Mobile Documents/com~apple~CloudDocs/Nvidia/Aerial/cuBB-25-2-Codex"
UP="$LOCAL/upgrade-25-3-workspace"
git diff --no-index --name-only "$LOCAL/cuPHY" "$UP/cuPHY" > /tmp/cuPHY_diff_files.txt
```

### 3) Inspect one file delta (unified patch)
```bash
LOCAL="/Users/yschae87/Library/Mobile Documents/com~apple~CloudDocs/Nvidia/Aerial/cuBB-25-2-Codex"
UP="$LOCAL/upgrade-25-3-workspace"
git diff --no-index "$LOCAL/cuPHY/src/cuphy/pdsch_dmrs/pdsch_dmrs.cu" "$UP/cuPHY/src/cuphy/pdsch_dmrs/pdsch_dmrs.cu"
```

### 4) Port one file from 25.3 baseline into a staging branch
```bash
cp "$UP/cuPHY/src/cuphy/pdsch_dmrs/pdsch_dmrs.cu" "$LOCAL/cuPHY/src/cuphy/pdsch_dmrs/pdsch_dmrs.cu"
```

### 5) Build/test gate after each small batch
```bash
cd "/Users/yschae87/Library/Mobile Documents/com~apple~CloudDocs/Nvidia/Aerial/cuBB-25-2-Codex"
cmake -Bbuild -GNinja -DCMAKE_TOOLCHAIN_FILE=cuPHY/cmake/toolchains/native -DNVIPC_FMTLOG_ENABLE=OFF -DASIM_CUPHY_SRS_OUTPUT_FP32=ON
cmake --build build -t _pycuphy pycuphycpp
```

## Recommended Working Method
1. Create a dedicated branch for migration work only.
2. Migrate one track at a time (Track 1 -> Track 2 -> Track 3).
3. Keep commits small and topic-focused (e.g., `cuPHY: sync pdsch_dmrs with 25.3.2`).
4. After each commit: compile, run targeted tests, then continue.
5. Keep local analysis markdown files out of production sync commits.

## Immediate Next Step
- Start with `cuPHY/src/cuphy/pdsch_dmrs/pdsch_dmrs.cu` and `cuPHY/src/cuphy_channels/pdsch_tx.cpp` in a dedicated branch, then run a build gate before touching `cuPHY-CP` FH-generator code.
