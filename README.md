# GRACE

GRACE is a unified 2D multi-robot path-planning simulator and benchmark for
grid, roadmap, and continuous environments. It provides a shared backend,
graphical frontend, planner-plugin interface, format-conversion tools, and
repeatable benchmarking workflows.

This repository is the companion code for:

```bibtex
@inproceedings{11695963,
  author    = {Zang, Chuanlong and Mannucci, Anna and Barz, Isabelle and Schillinger, Philipp and Lier, Florian and Hönig, Wolfgang},
  title     = {{GRACE}: A Unified {2D} Multi-Robot Path Planning Simulator \& Benchmark for Grid, Roadmap, And Continuous Environments},
  booktitle = {2026 IEEE International Conference on Robotics and Automation (ICRA)},
  year      = {2026},
  pages     = {6423--6430},
  doi       = {10.1109/ICRA57385.2026.11695963}
}
```

## Quick start: native grid planners

The following Ubuntu 24.04 workflow is the shortest end-to-end path from a
fresh clone to a planner run. Replace the example repository URL with the
published GRACE URL.

The standard grid setup uses LaCAM0. CBSH2-RTC, EECBS, and MAPF-LNS2 require
the separate [USC planner opt-in](#optional-usc-licensed-grid-planners).

```bash
https://github.com/boschresearch/GRACE
cd GRACE

sudo apt-get update
sudo apt-get install -y --no-install-recommends \
  build-essential ca-certificates cmake git ninja-build \
  libboost-all-dev libeigen3-dev libfmt-dev libgtest-dev nlohmann-json3-dev \
  libompl-dev libpugixml-dev libyaml-cpp-dev

git submodule update --init --recursive \
  backend/extern/classical/lacam0

cmake --preset ci-planners-grid
cmake --build --preset ci-planners-grid
ctest --preset ci-planners-grid

out/build/ci-planners-grid/backend/grace-backend grid \
  --planner lacam0-src \
  --take-k 2 \
  --results-dir results/quick-start
```

The last command uses the tracked demo map and scenario. It writes
`metrics_summary.csv`, `state_trace.csv`, and a timestamped run log below the
selected results directory.

## Publication status

GRACE is a research prototype published as part of the paper above. This
publication snapshot is provided to support reproduction and further research;
it is not a supported product and is not actively maintained or monitored.

## Features

- One backend model for grid, roadmap, and continuous 2D environments.
- A graphical workflow for environment editing, agent setup, planning,
  simulation, metrics, and cross-environment comparison.
- Built-in planner profiles plus a C ABI plugin interface for external planners.
- MovingAI, XML/GraphML, and YAML conversion tools.
- Batch benchmarking with repeatable JSON configuration and CSV output.
- CMake presets and test lanes for fast, frontend, and planner integration
  validation.

The backend does not depend on the frontend. The frontend consumes backend
interfaces and services, so command-line simulation, conversion, and
benchmarking remain independently usable.

## Repository layout

| Path | Purpose |
| --- | --- |
| `backend/inc`, `backend/src` | Environment, planner, simulation, I/O, and benchmarking backend |
| `backend/data` | Example grid, roadmap, and continuous problems |
| `backend/benchmarking` | Benchmark configurations and plotting support |
| `frontend` | SFML/ImGui graphical application |
| `cmake` | Dependency, warning, and planner-profile configuration |
| `backend/extern`, `frontend/extern` | Optional third-party submodules and imported source |
| `CMakePresets.json` | Reproducible build and test profiles |

## Clone the repository

GRACE uses Git submodules for the frontend libraries and bundled planners. A
source archive downloaded from a hosting site does not contain those submodule
repositories. Use Git for planner or frontend builds.

For all standard components, clone recursively:

```bash
git clone --recurse-submodules \
  https://github.com/OWNER/REPOSITORY.git GRACE
cd GRACE
git submodule status --recursive
```

For a smaller download, clone normally and initialize only the paths required
by the chosen build, as shown in the build sections below. If the repository
was already cloned without submodules, initialize the standard components with:

```bash
git submodule update --init --recursive
```

The three USC-licensed solver submodules have `update = none` in `.gitmodules`.
On a fresh checkout, both commands above skip fetching them. Their pinned Git
references, GRACE-owned adapters, and plugin registry metadata remain in the
repository; the solver implementations are obtained only through explicit
opt-in. Git's `--checkout` option overrides this policy, so do not add it to a
general recursive setup command unless you intend to obtain the restricted
solvers as well.

The parent repository stores only a commit reference for each submodule.
Untracked files inside a submodule, including model weights and db-CBS motion
data, are not included when the parent repository is pushed.

## Install dependencies

GRACE currently has no `cmake --install` rules. “Install GRACE” therefore means
installing its dependencies, configuring a build tree, building it, and running
the executables from that build tree.

### Common backend dependencies

On Ubuntu 24.04:

```bash
sudo apt-get update
sudo apt-get install -y --no-install-recommends \
  build-essential ca-certificates cmake git ninja-build \
  libboost-all-dev libfmt-dev libgtest-dev nlohmann-json3-dev libompl-dev \
  libpugixml-dev libyaml-cpp-dev
```

The native grid profile also requires Eigen:

```bash
sudo apt-get install -y --no-install-recommends libeigen3-dev
```

The presets download checksum-verified Box2D, SFML, Dear ImGui, and related
archives when a compatible system package is not selected. Consequently, the
first configure requires network access unless the archives already exist in
`.cache/downloads`.

### Graphical frontend

Install the common dependencies and:

```bash
sudo apt-get install -y --no-install-recommends \
  libfreetype6-dev libgl1-mesa-dev libudev-dev libx11-dev \
  libxcursor-dev libxi-dev libxrandr-dev
```

### Continuous planners

db-CBS and SST use the db-CBS/dynoplan dependency tree. In addition to the
common packages, install:

```bash
sudo apt-get install -y --no-install-recommends \
  coinor-libipopt-dev curl libccd-dev libeigen3-dev libfcl-dev \
  liblz4-dev libmsgpack-dev unzip
```

They also require Crocoddyl and its dependencies. Follow the
[Crocoddyl installation guide](https://github.com/loco-3d/crocoddyl#installation)
and make its CMake package discoverable. Robotpkg installs are normally below
`/opt/openrobots`, which is the default value of `GRACE_OPENROBOTS_PREFIX`.
For a different installation, pass the prefix during configuration:

```bash
cmake --preset full -DGRACE_OPENROBOTS_PREFIX=/path/to/openrobots
```

If shared libraries are installed below `/opt/openrobots`, they may also need
to be available at runtime:

```bash
export LD_LIBRARY_PATH="/opt/openrobots/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
```

db-CBS additionally requires motion primitives that are not tracked by its
submodule. After initializing `backend/extern/continuous_space/db-CBS`, download
the archive referenced by that submodule:

```bash
(
  cd backend/extern/continuous_space/db-CBS
  curl -fL \
    https://tubcloud.tu-berlin.de/s/CijbRaJadf6JwH3/download \
    -o /tmp/dbcbs-motions.zip
  unzip /tmp/dbcbs-motions.zip
)

test -f \
  backend/extern/continuous_space/db-CBS/motions/double_integrator_0_sorted.msgpack

(
  cd backend/extern/continuous_space/db-CBS
  sha256sum --check <<'EOF'
6675d82f144555a56633cf5cc800b3a896401af4e1d4902e7707cebd08dbd82b  motions/car_first_order_with_1_trailers_0_sorted.msgpack
66b6a39765d554105d9ecd6b1bd2244673568e116c749871fe8936338d83454e  motions/double_integrator_0_sorted.msgpack
a0eacc7eb17ddcbfd3e26ebec0840689951d27e14c08aaa75f213a91eadfd7a7  motions/unicycle_first_order_0_sorted.msgpack
decf849c1b553fa141fb4eec4d6ae9d45aaf017bf4e6f6db6435fa13ac5ab825  motions/unicycle_second_order_0_sorted.msgpack
EOF
)
```

Without this data, `dbcbs-bin`, `dbcbs-src`, and the default continuous example
fail at runtime. SST does not load that motion file, but shares the continuous
build dependencies.

### Docker-backed learned planners

`mapfgptddg-bin` and `scrimp-bin` run Python implementations in local Docker
images. Install [Docker Engine](https://docs.docker.com/engine/install/) and
ensure the current user can run `docker` without `sudo`, because GRACE invokes
the Docker CLI directly.

After initializing both submodules, build the images using their lowercase
Dockerfile names:

```bash
docker build \
  -f backend/extern/classical/MAPF-GPT-DDG/docker/dockerfile \
  -t mapf-gpt \
  backend/extern/classical/MAPF-GPT-DDG/docker

docker build \
  -f backend/extern/classical/SCRIMP/docker/dockerfile \
  -t scrimp \
  backend/extern/classical/SCRIMP/docker
```

MAPF-GPT-DDG downloads `hf_weights/model-2M-DDG.pt` from Hugging Face on first
use, so that run requires network access and writes the cache into its
bind-mounted submodule.

SCRIMP does not download its checkpoint automatically. Download the fully
trained model linked from
[`backend/extern/classical/SCRIMP/README.md`](backend/extern/classical/SCRIMP/README.md)
and install it at the path expected by the adapter:

```bash
mkdir -p backend/extern/classical/SCRIMP/final
curl -fL \
  'https://www.dropbox.com/scl/fo/ekhxyt7gm575kfwaerwb5/h?rlkey=j3cdikwofz0zelj2oci9q97k8&dl=1' \
  -o /tmp/scrimp-final.zip
unzip -j /tmp/scrimp-final.zip net_checkpoint.pkl \
  -d backend/extern/classical/SCRIMP/final

echo \
  'fe7604932a3b3627265e864e5c96054ddfd0fff9ee9bfcc58dd39a360bf73855  backend/extern/classical/SCRIMP/final/net_checkpoint.pkl' \
  | sha256sum --check
```

Verify the runtime assets before testing:

```bash
docker image inspect mapf-gpt scrimp >/dev/null
test -f backend/extern/classical/SCRIMP/final/net_checkpoint.pkl
```

Both learned planners default to `device=cuda` when called from the backend,
frontend, or benchmark executable. Those entry points therefore require an
NVIDIA GPU and the
[NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html).
The planner integration tests contain explicit CPU cases, but the public
command-line interface does not currently expose a planner-device option.

## Build and test

Choose the smallest build that contains the required application and planner.
Build and test presets use at most four workers. Build trees are written below
`out/build`, and downloaded dependency archives are cached below
`.cache/downloads`.

### Available planner profiles

`GRACE_PLANNER_PROFILE` controls which bundled planner artifacts are built.
Registry metadata for external plugins remains available regardless of profile.

| Profile | Bundled planners |
| --- | --- |
| `NONE` | No bundled planner artifacts |
| `GRID` | LaCAM0 |
| `ROADMAP` | CCBS, PSIPP-CTC |
| `CONTINUOUS` | db-CBS, SST |
| `HOSTED` | LaCAM0, CCBS, PSIPP-CTC |
| `FULL` | All standard planners, including the Docker adapters; excludes USC planners |
| `CUSTOM` | Individual `GRACE_BUILD_PLANNER_<NAME>` switches |

A plain CMake configure defaults to `NONE`. The `FULL` profile builds the C++
adapters for MAPF-GPT-DDG and SCRIMP; their Docker images and model files remain
separate runtime prerequisites.

`GRACE_ENABLE_USC_PLANNERS` defaults to `OFF`, including in every standard
preset. Explicitly setting it to `ON` adds CBSH2-RTC, EECBS, and MAPF-LNS2 to
`GRID`, `HOSTED`, and `FULL`. In `CUSTOM`, the corresponding individual switches
also require this opt-in. It does not download source code or enable these
planners in `NONE`, `ROADMAP`, or `CONTINUOUS`.

### Core backend, without bundled planners

This lane needs no submodules:

```bash
cmake --preset ci-fast
cmake --build --preset ci-fast
ctest --preset ci-fast
```

It builds `grace-backend` for help and service validation, plus the conversion
tool. Simulation commands cannot use a bundled planner from this build.

### Frontend and service integration, without bundled planners

```bash
git submodule update --init --recursive \
  frontend/extern/enkiTS \
  frontend/extern/jsmn

cmake --preset ci-integration
cmake --build --preset ci-integration
ctest --preset ci-integration
```

This profile validates the GUI and services but uses planner profile `NONE`.
The application can start, but planning requires an external plugin or a build
tree configured with bundled planners.

### Native grid planners

```bash
git submodule update --init --recursive \
  backend/extern/classical/lacam0

cmake --preset ci-planners-grid
cmake --build --preset ci-planners-grid
ctest --preset ci-planners-grid
```

### Optional USC-licensed grid planners

CBSH2-RTC, EECBS, and MAPF-LNS2 are not fetched or built by the standard setup.
Review the licenses at the pinned revisions before choosing to obtain them:

| Solver | License |
| --- | --- |
| CBSH2-RTC | [USC license](https://github.com/Chuanlong-Zang/CBSH2-RTC/blob/70cdcbfef0cbf761415c617d7272e845db9c620a/license.md) |
| EECBS | [USC license](https://github.com/Chuanlong-Zang/EECBS/blob/6b610915fc5cfb1f68c38c5f0e71ab7dd6360553/license.md) |
| MAPF-LNS2 | [USC license and PIBT notice](https://github.com/Chuanlong-Zang/MAPF-LNS2/blob/30d38d81280072aec63c29444e9fba2cf3d7f9e2/license.txt) |

To fetch these optional solvers, run this separate command from the repository
root. The targeted `--checkout` overrides `update = none` for this invocation
and checks out the pinned commits; it does not change the default policy:

```bash
git submodule update --init --recursive --checkout -- \
  backend/extern/classical/CBSH2-RTC \
  backend/extern/classical/EECBS \
  backend/extern/classical/MAPF-LNS2
```

After initializing LaCAM0 as shown above, build in the separate opt-in tree:

```bash
cmake --preset usc-grid
cmake --build --preset usc-grid
ctest --preset usc-grid

out/build/usc-grid/backend/grace-backend grid \
  --planner eecbs-src --take-k 2 --results-dir results/eecbs-src
```

For a custom build containing only one of these solvers, fetch only its path
and set both `-DGRACE_ENABLE_USC_PLANNERS=ON` and its
`-DGRACE_BUILD_PLANNER_<NAME>=ON` switch with `GRACE_PLANNER_PROFILE=CUSTOM`.
The same opt-in flag can extend `HOSTED` or `FULL`; use a separate build directory
for those opt-in artifacts too.

> [!IMPORTANT]
> The USC texts permit use, copying, modification, and distribution for
> educational, research, and non-profit purposes subject to their terms;
> commercial use requires separate permission. This technical opt-in does not
> grant additional rights, relicense the solvers, or establish compatibility
> with GRACE's AGPL-3.0-only license. In particular, `*-src` plugins link solver
> code. Obtain appropriate legal review or permissions before redistributing
> combined artifacts or using these solvers outside the permitted purposes.

Previously downloaded sources, local Git configuration overrides, and old build
artifacts are not removed automatically. Standard presets reset the build
opt-in to `OFF`, but do not delete stale solver libraries. Use a fresh checkout
and a fresh standard build directory for distributions, not a previously
opted-in working tree.

### Native roadmap planners

```bash
git submodule update --init --recursive \
  backend/extern/roadmap/CCBS \
  backend/extern/roadmap/PSIPP-CTC

cmake --preset ci-planners-roadmap
cmake --build --preset ci-planners-roadmap
ctest --preset ci-planners-roadmap
```

### Continuous planners

There is currently no supplied continuous preset. After installing the
continuous dependencies, initializing db-CBS recursively, and downloading its
motion data, configure a dedicated build tree:

```bash
git submodule update --init --recursive \
  backend/extern/continuous_space/db-CBS

cmake -S . -B out/build/continuous -G Ninja \
  -DCMAKE_BUILD_TYPE=Release \
  -DGRACE_BUILD_BACKEND=ON \
  -DGRACE_BUILD_FRONTEND=OFF \
  -DGRACE_BUILD_TESTS=ON \
  -DGRACE_BUILD_PLANNER_INTEGRATION_TESTS=ON \
  -DGRACE_BUILD_BENCHMARKS=OFF \
  -DGRACE_PLANNER_PROFILE=CONTINUOUS \
  -DGRACE_USE_SYSTEM_BOX2D=OFF \
  -DGRACE_USE_SYSTEM_FMT=ON \
  -DGRACE_USE_SYSTEM_GTEST=ON \
  -DGRACE_OPENROBOTS_PREFIX=/opt/openrobots \
  -DGRACE_DEPENDENCY_DOWNLOAD_DIR="$PWD/.cache/downloads"

cmake --build out/build/continuous -j4
ctest --test-dir out/build/continuous \
  -L planner-continuous \
  --output-on-failure
```

### Build one selected planner

Use `CUSTOM` when only one planner is needed. For example, this creates a
backend and benchmark build containing LaCAM0:

```bash
git submodule update --init --recursive \
  backend/extern/classical/lacam0

cmake -S . -B out/build/lacam0 -G Ninja \
  -DCMAKE_BUILD_TYPE=Release \
  -DGRACE_BUILD_BACKEND=ON \
  -DGRACE_BUILD_FRONTEND=OFF \
  -DGRACE_BUILD_TESTS=ON \
  -DGRACE_BUILD_PLANNER_INTEGRATION_TESTS=ON \
  -DGRACE_BUILD_BENCHMARKS=ON \
  -DGRACE_PLANNER_PROFILE=CUSTOM \
  -DGRACE_BUILD_PLANNER_LACAM0=ON \
  -DGRACE_USE_SYSTEM_BOX2D=OFF \
  -DGRACE_USE_SYSTEM_FMT=ON \
  -DGRACE_USE_SYSTEM_GTEST=ON \
  -DGRACE_DEPENDENCY_DOWNLOAD_DIR="$PWD/.cache/downloads"

cmake --build out/build/lacam0 -j4
ctest --test-dir out/build/lacam0 --output-on-failure
```

Change both the submodule path and the
`GRACE_BUILD_PLANNER_<NAME>` switch using the planner table below.

### Full standard build

The full profile includes the continuous and learned planners and their runtime
requirements, but still excludes the three USC-licensed solvers:

```bash
git submodule update --init --recursive

cmake --preset full -DGRACE_OPENROBOTS_PREFIX=/opt/openrobots
cmake --build --preset full
```

The full test suite always exercises the Docker-backed planners, even when the
explicit GPU tests are skipped. It therefore requires Docker, both images, the
SCRIMP checkpoint, and MAPF-GPT-DDG network access or a cached weight.

On a CPU-only Docker host, exclude the tests that inherit the planners' CUDA
defaults:

```bash
ctest --preset full \
  -E 'PlannersGPU|PlannersSequential.SwapSameSimulationInstance'
```

On an NVIDIA host with the Container Toolkit, run the full suite and then the
explicit CUDA cases:

```bash
ctest --preset full
GRACE_RUN_GPU_TESTS=1 ctest --test-dir out/build/full \
  -R 'PlannersGPU' --output-on-failure
```

## Select and run a planner

Planner identifiers are lowercase and environment-specific. Passing a grid
planner to a roadmap or continuous run is rejected.

| Environment | Planner | CMake switch suffix | Runtime identifier(s) | Submodule |
| --- | --- | --- | --- | --- |
| Grid | EECBS (USC opt-in) | `EECBS` | `eecbs-bin`, `eecbs-src` | `backend/extern/classical/EECBS` |
| Grid | CBSH2-RTC (USC opt-in) | `CBSH2RTC` | `cbsh2rtc-bin`, `cbsh2rtc-src` | `backend/extern/classical/CBSH2-RTC` |
| Grid | LaCAM0 (default) | `LACAM0` | `lacam0-bin`, `lacam0-src` | `backend/extern/classical/lacam0` |
| Grid | MAPF-LNS2 (USC opt-in) | `MAPFLNS2` | `mapflns2-bin`, `mapflns2-src` | `backend/extern/classical/MAPF-LNS2` |
| Grid | MAPF-GPT-DDG | `MAPFGPTDDG` | `mapfgptddg-bin` | `backend/extern/classical/MAPF-GPT-DDG` |
| Grid | SCRIMP | `SCRIMP` | `scrimp-bin` | `backend/extern/classical/SCRIMP` |
| Roadmap | CCBS | `CCBS` | `ccbs-bin`, `ccbs-src` | `backend/extern/roadmap/CCBS` |
| Roadmap | PSIPP-CTC (default) | `PSIPPCTC` | `psippctc-bin`, `psippctc-src` | `backend/extern/roadmap/PSIPP-CTC` |
| Continuous | db-CBS (default) | `DBCBS` | `dbcbs-bin`, `dbcbs-src` | `backend/extern/continuous_space/db-CBS` |
| Continuous | SST | `SST` | `sst-bin`, `sst-src` | `backend/extern/continuous_space/db-CBS` |

`*-bin` is a built-in C++ adapter that launches the planner's command-line
driver. `*-src` is a dynamically loaded C ABI plugin linked to the planner
core. Both forms are built by the corresponding switch. An external source
plugin can be selected by setting `GRACE_PLUGIN_DIR` to the directory containing
its `libplanner_<name>_src.so`.

Registry entries (and frontend planner names) do not imply that solver code is
present. External plugins retain their own licensing requirements.

Run tracked examples with explicit planner names:

```bash
out/build/ci-planners-grid/backend/grace-backend grid \
  --planner lacam0-src \
  --take-k 2 \
  --results-dir results/lacam0-src

out/build/ci-planners-roadmap/backend/grace-backend roadmap \
  --planner ccbs-src \
  --take-k 2 \
  --results-dir results/ccbs-src

out/build/continuous/backend/grace-backend continuous \
  --planner dbcbs-src \
  --take-k 1 \
  --results-dir results/dbcbs-src
```

Inspect all simulation options with:

```bash
out/build/ci-planners-grid/backend/grace-backend --help
```

If `--planner` is omitted, GRACE selects `lacam0-src`, `psippctc-src`, or
`dbcbs-src` according to the environment. That default still has to be present
in the selected build.

## Run the graphical frontend

For planning in the GUI, use `full` or a custom frontend build containing the
desired planners. Run the executable from the frontend source directory so its
relative font, shader, and settings paths resolve:

```bash
cd frontend
../out/build/full/frontend/grace_frontend
```

The frontend writes UI state to `settings.ini` and `imgui.ini` in its working
directory.

## Convert environment formats

The converter is planner-independent and is available from the `ci-fast`
build:

```bash
out/build/ci-fast/backend/grace_backend_convert --help
out/build/ci-fast/backend/grace_backend_convert --mode demo
backend/run_pipeline.sh \
  out/build/ci-fast/backend/grace_backend_convert
```

The pipeline writes converted roadmap, grid, scenario, and YAML artifacts below
`results/pipeline` by default. Set `GRACE_PIPELINE_OUTDIR` to change that
artifact directory:

```bash
GRACE_PIPELINE_OUTDIR=/tmp/grace-pipeline \
  backend/run_pipeline.sh \
  out/build/ci-fast/backend/grace_backend_convert
```

Converter invocations also create timestamped run logs below `results`.

## Benchmark planners

The tracked benchmark example is grid-only and compares `lacam0-bin` and
`lacam0-src`, so no USC opt-in is required. Enable benchmarks in the native grid
build before running it:

```bash
cmake --preset ci-planners-grid -DGRACE_BUILD_BENCHMARKS=ON
cmake --build --preset ci-planners-grid

out/build/ci-planners-grid/backend/grace_backend_benchmark \
  --config backend/benchmarking/configs/examples/inline.json
```

Results are written to `results/<session>/`, including an aggregate
`all_runs.csv`. Planner names in a JSON configuration use the same runtime
identifiers listed above. Keep each configuration environment-specific: every
configured solver is scheduled against every configured problem.

To generate plots and a LaTeX summary on Ubuntu:

```bash
sudo apt-get install -y --no-install-recommends \
  python3-matplotlib python3-numpy python3-pandas

python3 backend/benchmarking/bench_plots.py \
  --csv results/SESSION/all_runs.csv \
  --out results/SESSION/plots
```

Replace `SESSION` with the timestamp printed by the benchmark executable.

## Dependency selection

Set `GRACE_USE_SYSTEM_BOX2D`, `GRACE_USE_SYSTEM_FMT`,
`GRACE_USE_SYSTEM_GTEST`, or `GRACE_USE_SYSTEM_FRONTEND_DEPS` to require
compatible installed packages instead of pinned archives. Boost, OMPL,
pugixml, yaml-cpp, and Crocoddyl are system dependencies.

## Publishing a reproducible snapshot

Before pushing a release and testing its README from a fresh clone:

1. Push every submodule commit first, and verify that each URL and pinned commit
   is publicly reachable.
2. Commit the updated submodule references in the parent repository.
3. Publish the db-CBS motion archive and SCRIMP checkpoint at stable locations,
   record checksums, and update the download instructions. They are currently
   external, untracked runtime assets.
4. Verify that the root and initialized submodules contain no accidental local
   changes:

   ```bash
   git status --short
   git submodule status --recursive
   ```

5. Repeat the chosen workflow in a separate clean clone. Do not validate a
   release only against an existing build tree or locally cached model files.
6. For a standard distribution, exclude `.git` directories, optional USC solver
   checkouts, and any previously built USC solver executables or plugins.
   Disabling a CMake option does not remove old artifacts. Submodule references
   in the parent repository are not the solver source code.

This repository snapshot does not contain a GitHub Actions workflow. The
continuous and learned-planner paths, as well as the opt-in USC planners, have
not been CI-validated. A release claiming those paths should validate them
separately on a clean machine with the documented external dependencies.

## License

GRACE-owned code is licensed under the
[GNU Affero General Public License v3.0 only](LICENSE).

Third-party components retain their original licenses and are not covered by
the GRACE license. Several optional planner integrations have additional
research/non-commercial restrictions. Consult
[`3rd-party-licenses.txt`](3rd-party-licenses.txt) before redistribution or
commercial use.
