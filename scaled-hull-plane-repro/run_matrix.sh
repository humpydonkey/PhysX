#!/bin/bash
# usage: run_matrix.sh <directory with libPhysXGpu_64.so> [path to repro binary]
# One line per run. regen=N is the number of manifold regenerations counted by the optional
# debug-count-regeneration.diff patch (it prints PXREGEN); it is 0 for a build without that patch.
set -e
export LD_LIBRARY_PATH="$1:$LD_LIBRARY_PATH"
BIN=${2:-./scaled_hull_plane_repro}
run() {
	out=$("$BIN" "$@")
	echo "$(echo "$out" | grep '^scale=') regen=$(echo "$out" | grep -c PXREGEN || true)"
}
echo "# tilt 1 degree: the unscaled hull settles"
for az in 20 60 120 200; do for s in 1 0.1 0.0625; do run tilt=1 azimuth=$az scale=$s; done; done
echo "# tilt 3 degrees"
for az in 0 20 35 60 90 120 160 200 250 300; do for s in 1 0.1 0.0625; do run tilt=3 azimuth=$az scale=$s; done; done
