#!/bin/bash
# Regenerates contrib/ from the two pinned submodules, so the "what did NTNU change"
# layer is reproducible from git alone rather than being a hand-maintained copy.
#
#   contrib/include/rovio/*.hpp  the ROVTIO-modified headers we actually compile,
#                                with the rovtio->rovio rename undone so they drop
#                                straight onto upstream via include-path precedence
#   contrib/patches/*.diff       the same thing expressed as diffs vs upstream
#
# ROS-coupled files are documented but never compiled: RovioNode.hpp, featureTracker.hpp,
# rovio_node.cpp, rovio_rosbag_loader.cpp, feature_tracker_node.cpp.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
UP="$ROOT/third_party/rovio"
FORK="$ROOT/third_party/rovtio-fork"
[ -d "$UP/include/rovio" ] && [ -d "$FORK/include/rovtio" ] || {
    echo "submodules missing: git submodule update --init --recursive" >&2; exit 1; }

norm() { sed 's/rovtio/rovio/g; s/ROVTIO/ROVIO/g; s/Rovtio/Rovio/g' "$1"; }

mkdir -p "$ROOT/contrib/include/rovio" "$ROOT/contrib/patches"
rm -f "$ROOT"/contrib/include/rovio/*.hpp "$ROOT"/contrib/patches/*.diff

COMPILED="FeatureManager ImagePyramid ImgUpdate ImuPrediction MultilevelPatch MultilevelPatchAlignment Patch"
ROSONLY_H="RovioNode featureTracker"
ROSONLY_C="rovio_node rovio_rosbag_loader feature_tracker_node"

same=0
for f in "$FORK"/include/rovtio/*.hpp; do
    b=$(basename "$f")
    [ -f "$UP/include/rovio/$b" ] || continue
    norm "$f" > /tmp/rt.$$
    cmp -s "$UP/include/rovio/$b" /tmp/rt.$$ && same=$((same+1))
done

for b in $COMPILED; do
    norm "$FORK/include/rovtio/$b.hpp" > "$ROOT/contrib/include/rovio/$b.hpp"
    diff -u "$UP/include/rovio/$b.hpp" "$ROOT/contrib/include/rovio/$b.hpp" \
        > "$ROOT/contrib/patches/$b.hpp.diff" || true
done
for b in $ROSONLY_H; do
    norm "$FORK/include/rovtio/$b.hpp" > /tmp/rt.$$
    diff -u "$UP/include/rovio/$b.hpp" /tmp/rt.$$ > "$ROOT/contrib/patches/ROSONLY_$b.hpp.diff" || true
done
for b in $ROSONLY_C; do
    norm "$FORK/src/$b.cpp" > /tmp/rt.$$
    diff -u "$UP/src/$b.cpp" /tmp/rt.$$ > "$ROOT/contrib/patches/ROSONLY_$b.cpp.diff" || true
done
rm -f /tmp/rt.$$

echo "contrib/ regenerated"
echo "  identical to upstream : $same headers"
echo "  compiled overlay      : $(ls "$ROOT"/contrib/include/rovio/*.hpp | wc -l | tr -d ' ') headers"
echo "  documented ROS-only   : $(ls "$ROOT"/contrib/patches/ROSONLY_* | wc -l | tr -d ' ') files"
