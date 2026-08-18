#!/bin/bash
# Assemble the compile tree from three clearly separated layers.
#
#   layer 1  third_party/rovio          pristine upstream ROVIO (git submodule, never edited)
#   layer 2  contrib/include/rovio      ROVTIO's contribution, 7 headers (never edited either --
#                                       this is exactly ntnu-arl's code, namespace-normalised)
#   layer 3  fixes/*.patch              OUR changes: modern toolchain + bug fixes
#
# Anything in the staged tree is layer1, overridden by layer2, then patched by layer3.
# Every layer's contribution therefore stays separately auditable in git.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUT="${1:-$ROOT/build/staged}"

rm -rf "$OUT"
mkdir -p "$OUT/include" "$OUT/src" "$OUT/shaders"

# -- layer 1: upstream ROVIO -------------------------------------------------
cp -R "$ROOT/third_party/rovio/include/rovio" "$OUT/include/rovio"
cp -R "$ROOT/third_party/lightweight_filtering/include/lightweight_filtering" "$OUT/include/"
cp -R "$ROOT/third_party/kindr/include/kindr" "$OUT/include/"
cp "$ROOT/third_party/rovio/shaders/"* "$OUT/shaders/"
for f in Camera FeatureCoordinates FeatureDistance Scene; do
    cp "$ROOT/third_party/rovio/src/$f.cpp" "$OUT/src/"
done
# ROS-only translation units are never compiled, so they are never staged:
#   RovioNode.hpp featureTracker.hpp rovio_node.cpp rovio_rosbag_loader.cpp feature_tracker_node.cpp
rm -f "$OUT/include/rovio/RovioNode.hpp" "$OUT/include/rovio/featureTracker.hpp"
L1=$(find "$OUT/include/rovio" -name '*.hpp' | wc -l | tr -d ' ')

# -- layer 2: ROVTIO contribution -------------------------------------------
L2=0
for h in "$ROOT"/contrib/include/rovio/*.hpp; do
    cp "$h" "$OUT/include/rovio/$(basename "$h")"
    L2=$((L2 + 1))
done

# -- layer 3: our fixes ------------------------------------------------------
L3=0
shopt -s nullglob
for p in "$ROOT"/fixes/*.patch; do
    patch -s -p1 -d "$OUT" < "$p"
    L3=$((L3 + 1))
done

echo "staged -> $OUT"
echo "  layer 1  upstream ROVIO headers : $L1"
echo "  layer 2  ROVTIO contribution    : $L2 (overrides)"
echo "  layer 3  our fixes              : $L3 patches"
