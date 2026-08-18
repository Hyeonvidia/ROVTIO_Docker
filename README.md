# ROVTIO, ROS-free, in Docker on Apple silicon

Runs [ntnu-arl/rovtio](https://github.com/ntnu-arl/rovtio) (visual + thermal + inertial odometry)
with **no ROS at all** — no roscpp, no rosbag, no cv_bridge, no param server — on `linux/arm64`,
with the built-in OpenGL viewer reachable from a browser.

Input is the already-extracted EuRoC-style dataset at
`/Users/jhpark/VSLAM/Datasets/rovtio/extracted/<seq>/`.

## Quick start

```bash
docker build --platform linux/arm64 -f docker/Dockerfile -t rovtio-rosfree .
./run.sh check           # config + calibration dry run
./run.sh batch  lt2      # headless, writes out/lt2.txt (TUM) + out/lt2_diag.csv
./run.sh gui    lt2      # real time; the browser opens on the viewer by itself
./run.sh native lt2      # same, but in macOS Screen Sharing instead of a browser
```

`gui` and `native` wait for the port and then hand the URL to macOS, so nothing has to be
opened by hand. `NO_OPEN=1 ./run.sh gui lt2` suppresses that. Host ports default to
**6081** (browser) and **5901** (Screen Sharing) — override with `ROVTIO_WEB_PORT` /
`ROVTIO_VNC_PORT`; see the caching note under Viewer for why they are not 6080/5900. Sequences: `lt2 alt1 alt2 lt1
lt3 sync`. Anything after the sequence name is passed to `rovtio_player`
(`./run.sh gui lt2 --max-seconds 60 --viewer-hz 10`).

Evaluate against the LOAM reference:

```bash
python3 scripts/evaluate.py out/lt2.txt \
  /Users/jhpark/VSLAM/Datasets/rovtio/extracted/lt2/aft_mapped_to_init_CORRECTED_tum.txt \
  --max-diff 0.1
```

## Measured result (lt2, 162 s, 99.6 m reference path)

| run | poses | APE rmse | APE / path | est. path |
|---|---:|---:|---:|---:|
| visual only (`ROVIO_NCAM=1`) | 3209 | 0.271 m | 0.27 % | 105.3 m |
| visual + thermal (`ROVIO_NCAM=2`) | 7534 | 0.280 m | 0.28 % | 107.5 m |

SE(3) Umeyama alignment, no scale fitting. Reference: `aft_mapped_to_init_CORRECTED` (LOAM).
Thermal features are active in 1212 of 7534 updates (up to 15 at once) — lt2 has usable
visual throughout, so thermal contributes intermittently rather than continuously.

Fitting scale as a diagnostic gives **0.927**, i.e. the estimate runs ~7.9 % long. The shipped
`rovtio.info` sets `Prediction.GravityVector.g_z = -10.0854` against a measured stationary
|a| of 10.05 m/s², so part of that is a deliberate absorber for this VN-100's accelerometer
scale error rather than a bug — but treat absolute scale as approximate.

Throughput: ~3.5 ms per filter step on an M4 Max under llvmpipe, so the 200 Hz IMU is real-time
with margin. A full 162 s sequence replays headless in well under a minute.

## Repository layout — three separable layers

The point of this structure is that **what NTNU changed and what we changed are both visible
as diffs**, and upstream ROVIO is never edited.

```
third_party/rovio                 git submodule, ethz-asl/rovio @34735c0 -- PRISTINE
third_party/kindr                 git submodule, header-only
third_party/lightweight_filtering git submodule, header-only
contrib/include/rovio/*.hpp       ROVTIO's contribution: 7 headers, namespace-normalised
contrib/patches/*.diff            the same contribution expressed as diffs vs upstream
fixes/*.patch                     OUR changes: modern toolchain + genuine bug fixes
rosfree/                          the ROS-free driver, dataset player and CMake build
cfg/                              rovtio_rosfree.info + camera calibrations
scripts/stage_sources.sh          assembles the compile tree from the three layers
```

`scripts/stage_sources.sh` copies upstream, overlays `contrib/`, then applies `fixes/`.
Re-running it from scratch reproduces the tree byte-for-byte.

### How much of ROVTIO is actually new

Comparing every source file against upstream ROVIO (after normalising the `rovtio` →
`rovio` rename): **28 of 38 files are byte-identical**, no files are added or removed, and
10 are modified.

| file | +/− | note |
|---|---|---|
| `ImgUpdate.hpp` | +1313 / −918 | the real contribution: multi-modality fusion |
| `RovioNode.hpp` | +392 / −155 | multi-camera ROS plumbing — **dropped here** |
| `MultilevelPatchAlignment.hpp` | +139 / −139 | |
| `Patch.hpp` | +86 / −8 | |
| `ImagePyramid.hpp` | +76 / −29 | |
| `FeatureManager.hpp` | +66 / −26 | |
| `MultilevelPatch.hpp` | +8 / 0 | |
| `ImuPrediction.hpp` | +3 / −1 | |
| `rovio_node.cpp`, `rovio_rosbag_loader.cpp` | +8 / −3 | ROS entry points — **dropped here** |

Dropping the three ROS files leaves **7 headers** to overlay. Upstream HEAD is the correct
submodule pin: for 7 of the 8 modified headers, HEAD is already the closest upstream version,
so the overlay does not silently revert later upstream work.

## What we had to fix (`fixes/`)

Two of these are real defects, not just modernisation.

1. **`02-rovioscene-heap-overrun`** — ROVTIO rewrote `Patch::drawPatch` to write through a
   `float*` (upstream writes `uint8_t*`), but `RovioScene::patch_` is still allocated
   `CV_8UC1`. 2304 floats go into a 2304-**byte** buffer: a 4× heap overrun for every drawn
   feature on every frame, whenever the viewer is on. Fixed to `CV_32FC1` plus an explicit
   8-bit conversion for the texture upload (thermal patches carry raw counts and would
   otherwise saturate to white).
2. **`04-exceptions-dangling-cstr`** — `ExceptionBase::what()` returned `.c_str()` of a
   temporary that dies at the end of the expression. Undefined behaviour every time a ROVIO
   exception is printed. Now cached in a member.
3. `01-opencv4-cvtcolor` — `CV_GRAY2RGB` was removed in OpenCV 4.
4. `03-glut-depth-aspect-exit` — `Scene::init` enables `GL_DEPTH_TEST` but `initGlut` never
   requested a depth buffer; the window was 1600×900 while the projection assumed 1280×960
   (there is no `glutReshapeFunc` anywhere), so the view was squashed. Also makes window-close
   return from the main loop instead of `exit()`-ing and truncating the trajectory file.
5. `05-shader-essl3-texture` — `texture2D()` is not valid in `#version 300 es`.
6. `06-lwf-modern-cxx` — an illegal `.template` on a non-template member (clang rejects it),
   a header-scope `static` function, and `static_cast<TYPE>(0)` which is deleted for
   `std::string` in C++23.

## Deviations from the original that you should know about

**`[D1]` camera-inactivity uses the data clock, not the wall clock.**
`RovioNode` decides a camera has gone silent by comparing `ros::Time::now()` deltas, but
`canAddImage()` refuses to advance until every *active* camera has a queued frame. Replaying
faster than real time makes those wall-clock deltas ~0, so during a 936 ms Tau2 FFC shutter
the camera never gets marked inactive and the whole pipeline stalls for the entire gap — 1×
on lt2, 12× on lt1. The driver drives this off the dataset timestamps instead, which also
makes runs reproducible at any replay rate. See `rosfree/src/RovtioDriver.hpp`.

**`CalibrationFile` had to be filled in.** The shipped `cfg/rovtio/rovtio.info` leaves both
`CalibrationFile` entries empty; the ROS launch file supplied them via `camera{0,1}_config`.
With an empty string `RovioFilter::refreshProperties()` silently skips `Camera::load`, leaving
K = identity and RADTAN with zero distortion, and the filter then runs on nonsense without
complaining. `cfg/rovtio_rosfree.info` sets them, and the player refuses to start if any
camera's focal length is implausible.

**Thermal images are fed as raw 16-bit counts** in a `CV_32FC1`, never normalised.
`ImgUpdate` keys its own histogram equalisation off `max > 255`, so pre-scaling to 0–255 would
silently disable it. `IMREAD_GRAYSCALE` would be worse still — it right-shifts to 8 bits.

## Viewer

The container runs its own X server with Mesa's llvmpipe and exports it over VNC, reachable
either from a browser or from macOS Screen Sharing:

```
Xvfb :99 -screen 0 2200x1300x24 +extension GLX +render   ->  llvmpipe, GL 4.5 compatibility
x11vnc -display :99 -rfbport 5900
websockify --web=/usr/share/novnc 6080 localhost:5900    ->  http://localhost:6081/vnc.html
                                                         ->  vnc://localhost:5901 (Screen Sharing)
```

Verified end to end: the websocket upgrade returns `101 Switching Protocols`, the
`RFB 003.008` greeting arrives through the proxy, and every script `vnc.html` pulls in
returns 200.

**Two things guard against a stale browser cache**, which otherwise shows up as
`TypeError: null is not an object (evaluating 'document.getElementById("noVNC_mouse_button0")...')`
or similar, from `app/ui.js`. The Ubuntu `novnc` package stamps its files 2021-10-22 and
websockify sends no `Cache-Control`, so a browser's heuristic freshness — 10 % of the file's
age — is about **176 days**: it will run a cached `app/ui.js` without even revalidating. If any
other project ever served a different noVNC on the same host port, *that* JS executes against
our `vnc.html` and dies on an element it cannot find. So:

1. the image restamps `/usr/share/novnc` at build time, dropping heuristic freshness to ~0 so
   the browser revalidates every load;
2. the host ports are 6081/5901 rather than the conventional 6080/5900, which keeps this
   viewer in its own browser cache origin (and off macOS's own Screen Sharing port).

If you still see a stale-asset error, a hard reload (`Cmd+Shift+R`) clears it.

Verified in-container: `llvmpipe (LLVM 20.1.2)`, `OpenGL 4.5 (Compatibility Profile) Mesa 25.2.8`,
`GL_ARB_ES3_compatibility` present, freeglut available.

### Why not XQuartz

Measured, not assumed — XQuartz is installed here and was tested directly. It cannot show
either window type:

| | over XQuartz | needed |
|---|---|---|
| 3D Scene | `OpenGL version string: 1.4 (2.1 Metal)`, `GL_ARB_ES3_compatibility` absent | GL 4.3+ for the `#version 300 es` shaders |
| Tracker overlays | `No matching fbConfigs or visuals found` -> window never maps, `imshow` still returns cleanly | a GLX visual for Qt |

Root cause is one line of Mesa output: **`glx: failed to create drisw screen`**. Over a remote
X connection Mesa cannot bring up its software GLX screen, so `LIBGL_ALWAYS_SOFTWARE=1` does
not help and libGL falls back to indirect GLX — where all GL is executed by XQuartz's own
GL 1.4/2.1-Metal implementation. The tracker windows fail for a second, independent reason:
Ubuntu's OpenCV highgui is the Qt+OpenGL build, and Qt needs a GLX visual XQuartz will not
offer, so `cv::imshow` silently maps nothing.

`./run.sh native` is the answer to "I want native Mac windows, not a browser tab" — it points
macOS Screen Sharing at the same x11vnc. XQuartz needs no configuration for any of this; if
you experimented with `xhost`, `xhost -localhost` puts access control back.

Three windows appear:

- **Scene** — the 3D view: ground plane, a coordinate triad per camera at its estimated pose,
  per-feature patch quads and depth-uncertainty segments (green = tracked, red = in frame only,
  grey = neither), plus a blue trajectory trail we add. Drag to orbit; `q` quits.
- **Tracker0 / Tracker1** — the per-camera overlays, which are the more diagnostic view: the
  virtual-horizon HUD, predicted vs aligned feature positions, and rejection reasons drawn as
  text (`NIF` not in frame, `MD` Mahalanobis outlier, `INC` inconsistent, `PE` patch error).

## Versions

Ubuntu 24.04 arm64 with GCC 13.3, CMake 3.28.3, OpenCV 4.6, Eigen 3.4, yaml-cpp 0.8, Boost 1.83,
GLEW 2.2, Mesa 25.2. Nothing is built from source.

Two things deliberately are **not** on the newest version:

- **CMake stays at Ubuntu's 3.28.** CMake ≥ 4 removed compatibility with
  `cmake_minimum_required(<3.5)`, which all three vendored `CMakeLists.txt` still declare.
  Our own build declares 3.16 and never configures theirs, but adding the Kitware repo would
  break anyone who does.
- **`-march=native` is not used.** On aarch64 it resolves to whatever the build host exposes —
  under Docker on Apple silicon that includes `i8mm`/`bf16`, which SIGILLs on a Jetson Orin.
  The default is `-march=armv8.2-a+fp16+dotprod`; `-DROVTIO_NATIVE_ARCH=ON` opts back in.

## Cloning

```bash
git clone https://github.com/Hyeonvidia/ROVTIO_Docker.git
cd ROVTIO_Docker
git submodule update --init          # NOT --recursive, see below
```

**Do not use `--recursive`.** Both `third_party/rovio` and `third_party/rovtio-fork` declare
their own `lightweight_filtering` submodule, pinned at the same commit `0c8517a5` that
`third_party/lightweight_filtering` already provides. A non-recursive init leaves those two
nested copies empty (0 B) and the build uses our explicitly pinned one. Recursing just fetches
the same 936 KB twice more.
