# PDSCH Precoding Analysis (25.3, Source-Validated) (v1.1)

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

## Scope and Evidence
This update is based on:
- `pyaerial/src/aerial/phy5g/pdsch/pdsch_tx.py`
- `pyaerial/src/aerial/phy5g/config.py`
- `pyaerial/pybind11/pycuphy_pdsch_params.cpp`
- `pyaerial/pybind11/pycuphy_util.cpp`
- `cuPHY/src/cuphy_channels/pdsch_tx.cpp`
- `cuPHY/src/cuphy/pdsch_dmrs/pdsch_dmrs.cu`
- `cuPHY/src/cuphy/dl_rate_matching/dl_rate_matching.cu`
- `cuPHY/src/cuphy/cuphy_api.h`
- `pyaerial/tests/test_pdsch_pdsch_tx.py`
- `pyaerial/tests/test_pdsch_dmrs_tx.py`
- `pyaerial/tests/test_pdsch_separable_pdsch_tx.py`

## End-to-End Precoding Path
### 1) Python config layer
`_pdsch_config_to_cuphy()` in `config.py`:
- For each UE, precoding is enabled when `precoding_matrix` is non-empty.
- Sets:
- `enablePrcdBf=True`
- `pmwPrmIdx=<index into pmwPrms>`
- Appends `PmW(w=<matrix>, nPorts=<matrix.shape[1]>)`.

### 2) pybind translation
`pycuphy_pdsch_params.cpp`:
- Copies UE fields including `enablePrcdBf` and `pmwPrmIdx`.
- Sets cell-group `nPrecodingMatrices` and fills `pPmwPrms`.

`readPrecodingMatrix()` in `pycuphy_util.cpp`:
- Reads `np.complex64` matrix buffer.
- Uses column count as `nPorts`.
- Converts each complex value to `__half2` directly (`real -> x`, `imag -> y`).
- No normalization/scaling/conjugation is applied.

### 3) cuPHY setup/runtime
`PdschTx::prepareRateMatchingMultipleCells()` and `PdschTx::prepareDmrsMultipleCells()` pass:
- `precoding = (nPrecodingMatrices > 0)`

This is a cell-group-level switch. If any UE in the group is precoded, precoding-specialized kernel variants are selected for the whole group.

## API Data Model and Layout
From `cuphy_api.h`:
- Per UE:
- `cuphyPdschUePrm_t.enablePrcdBf`
- `cuphyPdschUePrm_t.pmwPrmIdx`
- Group-level matrix array:
- `cuphyPdschCellGrpDynPrm_t.nPrecodingMatrices`
- `cuphyPdschCellGrpDynPrm_t.pPmwPrms`

Matrix layout (`cuphyPmW_t`):
- `matrix[MAX_DL_LAYERS_PER_TB * MAX_DL_PORTS]`
- Row-major with UE layers as slower dimension and ports as columns.

## DMRS Behavior with Precoding
In `pdsch_dmrs.cu`:
- `cuphyUpdatePdschDmrsParams()` copies per-UE matrix into `PdschDmrsParams.pmW` when `enablePrcdBf==1`.
- Sets `Np` from the referenced `pPmwPrms[pmwPrmIdx].nPorts`.
- Validates `Np <= output tensor port dimension`; otherwise returns invalid argument.

Kernel note:
- DMRS fused kernel template specialization is chosen by group-level `enable_precoding`.
- Mixed groups (some UEs precoded, some not) still run the precoding-specialized kernel path.

## Rate-Matching/Modulation Behavior with Precoding
In `dl_rate_matching.cu`:
- Kernel function pointer is switched each setup based on group-level `precoding` flag.
- `fused_dl_rm_and_modulation<true>` uses shared-memory precoding matrix load and atomic accumulation to output ports.
- Even with precoding-specialized kernels active, per-TB logic checks `dmrs_params->enablePrcdBf` for no-precoding UE behavior.

Group-level impact:
- If one UE enables precoding, specialized kernels are chosen for all TBs in that cell group for the slot setup.

## Output Tensor Mapping Back to Caller
`PdschTx.cuphy_to_tx()` in `pdsch_tx.py`:
- No precoding matrix for UE: antenna indices derived from DMRS ports + SCID.
- Precoding matrix present: index width uses matrix column count (`shape[1]`).
- Final index set is built via `list(set(indices))`.

Implication:
- Index order is not explicitly stabilized (not sorted), so antenna-plane ordering can be non-deterministic.

## Runtime Guardrails and Error Paths
- `dl_rate_matching.cu` sets `CUPHY_PDSCH_STATUS_UNSUPPORTED_MAX_ER_PER_CB` when `er_max > PDSCH_MAX_ER_PER_CB_BITS`.
- `pdsch_tx.cpp` has explicit handling/logging for this status and can skip run for affected TB setup.
- `pdsch_tx.cpp::refCheckModulationMultipleCells()` skips QAM reference checks when any precoding matrix is enabled in group (expected, because post-precoding accumulation changes semantics).

## Test Coverage Status (Current)
Previous "minimal coverage" statements are stale.
Current tests do exercise precoding flows using vector-provided `tb*_PM_W` when available:
- `test_pdsch_pdsch_tx.py`
- `test_pdsch_dmrs_tx.py`
- `test_pdsch_separable_pdsch_tx.py`

These tests compare outputs against reference vectors and include both precoding-present and precoding-absent paths depending on vector content.

## Findings and Recommendations
1. Group-level kernel switch can over-apply expensive specialization.
- One precoded UE enables precoding-specialized kernels for all UEs in that cell group.
- Recommend evaluating per-UE-group or per-TB dispatch split for mixed workloads.

2. No power normalization guard in Python->pybind path.
- Input matrix values are copied verbatim to half precision.
- Recommend optional validation or normalization policy in `config.py` for safer usage.

3. Output antenna index ordering is unstable.
- `list(set(indices))` should be replaced with deterministic ordering (`sorted(set(indices))`) to avoid non-repeatable plane order.

4. Validation is dimension-centric, not semantic.
- Existing checks verify port count bounds, but not matrix condition/orthogonality/energy constraints.
- Recommend optional debug-mode matrix sanity checks for integration environments.
