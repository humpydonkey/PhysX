# Repro: scaled convex hull on a GPU plane

`scaled_hull_plane_repro.cpp` is one file. It uses only the public PhysX API.

It drops a convex prism (8 sides, radius 0.0584 m, height 0.279 m) on a `PxPlane`. The scene uses GPU dynamics, GPU broad phase, TGS, PCM and friction 0.6. The body starts at rest and tilted, with one rim edge on the plane. Gravity, the plane and friction cannot add energy, so the mechanical energy can only fall.

The program makes the same physical hull in two ways:

* `scale=1`: the points are cooked at physical size. The `PxMeshScale` is the identity.
* `scale=s`: the points are cooked `1/s` times larger. The `PxMeshScale` is `s`. The hull has the same shape, mass and inertia.

It prints one line per run:

| Field | Meaning |
|---|---|
| `gain_J` | The largest mechanical energy above the start energy. It is 0 when the energy only falls. |
| `end_minus_start_J` | The energy at the end minus the energy at the start. |
| `contact_set_changes` | The number of steps in which the set of hull vertices in contact changed. It is a lower bound for manifold regenerations. It uses the public API only. |
| `regen` | The number of manifold regenerations in the GPU plane kernel. It needs `debug-count-regeneration.diff`. It is 0 for a build without the patch. |

## Build

Build PhysX with GPU support (CUDA 12.8, Linux). Then build the repro.

```bash
cd physx
./generate_projects.sh linux-gcc
make -C compiler/linux-gcc-release -j"$(nproc)"
L=bin/linux.x86_64/release
g++ -O2 -std=c++14 -DNDEBUG -DPX_PHYSX_STATIC_LIB -Iinclude -I/usr/local/cuda/include \
  -o scaled_hull_plane_repro /path/to/scaled_hull_plane_repro.cpp \
  -Wl,--start-group $L/libPhysX_static_64.a $L/libPhysXExtensions_static_64.a $L/libPhysXCommon_static_64.a \
  $L/libPhysXFoundation_static_64.a $L/libPhysXPvdSDK_static_64.a -Wl,--end-group \
  -L/usr/local/cuda/lib64/stubs -lcuda -lpthread -ldl -lrt
```

The build of the GPU library is long. To make it shorter, set `PX_GENERATE_GPU_REDUCED_ARCHITECTURES` to `True` in `buildtools/presets/public/linux-gcc.xml`, and set the architecture list in `source/compiler/cmakegpu/CMakeLists.txt` to the architecture of your GPU.

## Run

```bash
# one run, with the GPU library of the build
LD_LIBRARY_PATH=$L ./scaled_hull_plane_repro tilt=1 azimuth=20 scale=0.0625

# the full table
bash run_matrix.sh $L ./scaled_hull_plane_repro
```

Options: `scale`, `azimuth` (deg), `tilt` (deg), `friction`, `sides`, `radius`, `height`, `steps`, `hz`, `pos_iters`, `vel_iters`, `speed`, `spin`, `gpu` (0 selects the CPU solver), `bodies` (more than one selects a timing run, no energy output).

To count the regenerations, apply `debug-count-regeneration.diff` (`git apply`) and rebuild. The patch prints `PXREGEN` from `convexPlaneNphase_kernel_core` for each regeneration of an existing manifold. `run_matrix.sh` counts these lines.

Three repeats of the same command give identical output.
