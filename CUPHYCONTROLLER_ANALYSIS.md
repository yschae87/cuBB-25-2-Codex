# cuPHY Controller Deep Dive

This report summarizes the `cuphycontroller` layer in `cuPHY-CP/cuphycontroller`: its role, configuration parsing, threading helpers, and integration with the broader cuPHY driver.

## Purpose & Scope
- Provides a **minimal PHY controller** that initializes the PHY driver, loads YAML configs, creates cells, and can simulate L2 traffic for testing.
- Bridges YAML configs → `phydriver_config`/`cell_phy_info` structs → `l1_*` driver calls.
- Optional standalone/L2-simulation entry points for quick bring-up.

## Key Entry Points
- **pc_init_phydriver** (`src/cuphydriver.cpp`): validates UL/DL worker lists and descriptors, calls `l1_init(pdh, ctx_cfg)`, then initializes OAM (`CuphyOAM::getInstance()->init_everything()`).
- **pc_finalize_phydriver**: wraps `l1_finalize(pdh)`.
- **pc_standalone_create_cells**: iterates over provided `cell_phy_info` vector, calling `l1_cell_create` then `l1_cell_start` per cell; logs errors per cell.
- **pc_start_l1**: placeholder to start the L1 driver (currently commented calls to `l1_start`).
- **pc_standalone_simulate_l2**: spins a generic worker thread (name `L2Adapter`) that cycles through slot commands, stamping SFN/slot, enqueuing `l1_enqueue_phy_work`, with coarse `wait_ns()` pacing.

## Configuration Parsing (YamlParser)
- **Files**: `src/yamlparser.cpp`, `include/yamlparser.hpp`.
- **parse_file(filename)**:
  - Loads YAML, checks SDK version (`aerial::check_yaml_version`).
  - Resolves `l2adapter_config_filename` via `CONFIG_YAML_FILE_PATH` + relative path.
  - Invokes `parse_cuphydriver_configs(root["cuphydriver_config"])` and `parse_cell_configs(...)`.
- **parse_standalone_config_file**: resolves `standalone_config_filename` if present.

### cuphydriver_config parsing highlights
- Populates `phydriver_config` with timing, worker, graph, and feature flags:
  - Validation/standalone, profiling window, slot count, Prometheus/log level, sched priorities.
  - Start section IDs (SRS/PRACH), enable UL/DL cuPHY graphs, CPU/GPU timeouts for UL order, C-plane disable, DPDK logging/prefix, accumulator TX scheduler knobs.
  - SM reservations per channel (`mps_sm_*` for PUSCH/PUCCH/PRACH/SRS/PDSCH/PDCCH/PBCH/GPU_COMMS/UL_ORDER), pdsch_fallback, GPU init comms (GPU via CPU/cpu_init_comms), green contexts, batched memcpy.
  - Timing/error thresholds: UL order GPU SRS timeout, SRS aggr launch offset, DL wait thresholds, sendCPlane_* backoffs.
  - Optional features: CQE tracing masks, OK testbench, EMPW disable, UL RX pkt tracing levels, PMU metrics, H2D copy thread affinity/priority, mMIMO_enable, enable_srs, aggr_obj_non_avail_th, UE mode + DL validation workers.
- Uses `has_key` + defaults with NVLOG warnings; returns -1 on critical parsing errors (e.g., invalid pusch_workCancelMode range).
- Validates backend metrics address (loopback host:port) and port availability when AERIAL_METRICS is enabled.

### cell_configs parsing (not shown in snippets)
- Reads NIC/GPU mappings, per-cell PRB stride, LDPC options, IQ format, timing offsets (T1a/Ta4), SCS (mu), section IDs, and beamforming settings. Populates `cell_phy_info` vector.
- Builds unique NIC list to avoid duplicating NIC entries across cells.

## Worker & L2 Simulation Helpers
- **l2_adapter_args**: holds slot command list, pacing (`usec`), number of slots.
- **pc_l2_adapter**: in a loop (finite or infinite based on `num_slots`), stamps SFN/slot into each cell of the active slot command, enqueues to `l1_enqueue_phy_work`, and spins `wait_ns` for crude slot pacing. Increments `slot_index` modulo available slot commands; rolls SFN (mod 256) and slot (mod 20 for μ=1).
- **wait_ns**: busy-wait loop with a small inline asm spin to minimize syscall overhead (non-sleeping delay).

## Integration Flow (happy path)
1) **Parse YAML**: `YamlParser::parse_file` → `parse_cuphydriver_configs` + `parse_cell_configs` → populate `phydriver_config`, `cell_phy_info`.
2) **Init driver**: `pc_init_phydriver` calls `l1_init` (sets up workers, tick, transports) then OAM init.
3) **Create/start cells**: `pc_standalone_create_cells` issues `l1_cell_create`/`l1_cell_start` per cell.
4) **(Optional) Simulate L2**: `pc_standalone_simulate_l2` spawns `L2Adapter` worker to enqueue rotating slot commands at a fixed interval.
5) **Run/Stop**: Worker threads drive DL/UL pipelines (outside cuphycontroller scope); `pc_finalize_phydriver` cleans up via `l1_finalize`.

## Notable Behaviors / Defaults
- Many YAML keys are optional; defaults are logged (NVLOGC/NVLOGW) to surface missing tuning knobs (e.g., `use_green_contexts`, `gpu_init_comms_via_cpu`, `h2d_copy_thread_cpu_affinity`).
- Metrics backend address enforcement: must be `127.0.0.1:PORT`, and port must be free; otherwise parsing fails.
- `pdsch_fallback`, SM reservations, and graph enables impact downstream cuPHY launch modes.
- UE mode adds DL validation worker list if provided.
- H2D copy thread config defaults to disabled; affinity defaults to CPU 29 if unspecified.

## Files & Anchors
- API wrappers: `include/cuphydriver.hpp`, `src/cuphydriver.cpp`
- YAML parsing: `include/yamlparser.hpp`, `src/yamlparser.cpp`
- Config examples: `config/*.yaml` (F08, P5G, nrSim, HARQ, etc.)
- Versioning: `version.txt`
- Example app: `examples/cuphycontroller_scf.cpp` (ties parser + driver init for SCF tests)

## Gaps / Considerations
- `pc_start_l1` currently stubbed; actual start handled elsewhere (`l1_start` commented). Ensure start/stop lifecycle matches driver expectations.
- Time pacing in `pc_l2_adapter` is coarse (busy-wait); for precise timing use the real tick generator instead of simulated pacing when integrating with NIC/FH.
- Error handling is mostly log-and-default; critical parse failures return -1 but many optional fields silently default—document required keys per deployment.
- Thread safety: parsing happens before runtime; runtime helpers rely on `l1_*` for thread-safe enqueue and lifecycle management.

