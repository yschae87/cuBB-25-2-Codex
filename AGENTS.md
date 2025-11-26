# Repository Guidelines

## Project Structure & Module Organization
- Root components: `cuPHY`, `cuPHY-CP`, `cuMAC`, `cuMAC-CP` host core C++/CUDA sources; shared CMake bits live in `cmake/`; vectors and harnesses sit in `testVectors` and `testBenches`.
- Python API work is in `pyaerial/` with packages under `src/aerial/`, bindings in `pybind11/`, helper scripts in `scripts/`, notebooks in `notebooks/`, and tests in `tests/`.

## Build, Test, and Development Commands
- Set `cuBB_SDK` to the repo root; build C++/bindings inside the dev or pyAerial container:
  - `cmake -Bbuild -GNinja -DCMAKE_TOOLCHAIN_FILE=cuPHY/cmake/toolchains/native -DNVIPC_FMTLOG_ENABLE=OFF -DASIM_CUPHY_SRS_OUTPUT_FP32=ON`
  - `cmake --build build -t _pycuphy pycuphycpp`
- Build/install the Python package in dev mode: `pyaerial/scripts/install_dev_pkg.sh`.
- Static analysis: `pyaerial/scripts/run_static_tests.sh` (flake8, pylint, mypy, interrogate).
- Unit tests: `pyaerial/scripts/run_unit_tests.sh`; requires GPU test vectors under `/mnt/cicd_tvs/develop/GPU_test_input/` or `TEST_VECTOR_DIR`.
- Containerized workflow: `pyaerial/container/build.sh` then `pyaerial/container/run.sh` for an ML-ready environment; launch notebooks from `pyaerial/notebooks`.

## Coding Style & Naming Conventions
- Python: follow PEP8, use type hints, and keep docstrings coverage-friendly for `interrogate`; modules lower_snake_case, classes CamelCase, constants UPPER_SNAKE.
- C++/CUDA: match local patterns in the module directories; prefer RAII, const-correctness, and the existing clang-format style where present.
- Keep changes focused and separated by module; avoid mixing host and device concerns in the same patch; surface configuration via env vars (`BUILD_ID`, `TEST_VECTOR_DIR`).

## Testing Guidelines
- Add or update Python tests in `pyaerial/tests` using `test_*.py` naming that mirrors module names.
- Pair new bindings or kernels with representative vectors; document GPU or large-data needs in test docstrings.
- Run static + unit suites before PRs; ensure new public APIs include docstrings and type hints.

## Commit & Pull Request Guidelines
- Commit messages: `<module>: imperative summary` (e.g., `pyaerial: fix ldpc decoding error`); keep logical changes in separate commits; do not commit vendor binaries or vectors.
- PRs: include scope, environment (container tag, GPU), and test commands/results; link issues; attach relevant logs/screens; update docs/notebooks when behavior changes.

## Security & Configuration Tips
- Keep secrets and proprietary test vectors out of git; rely on env vars and mounted paths instead.
- Prefer containerized builds and minimal privileges when validating scripts.
