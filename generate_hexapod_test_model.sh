#!/bin/bash
# ORBIT / TEST COPY -- run this ONCE on your machine (not in this sandbox,
# since gz/ign isn't installed here) to generate the static SDF model
# that world2.sdf includes via <uri>model://hexapod_test_model</uri>.
#
# Safe to delete this whole orbit_test setup once you're done --
# it does not touch any of your existing files.

set -e

PKG_SHARE=$(ros2 pkg prefix hexa_description)/share/hexa_description
OUT_DIR="$PKG_SHARE/models/hexapod_test_model"

mkdir -p "$OUT_DIR"

# 1. Process your real xacro with the same args your launch files use.
xacro "$PKG_SHARE/urdf/hexapod.urdf.xacro" \
    prefix:=hexa_ \
    package_name:=hexa_description \
    config_dir:="$PKG_SHARE" \
    > /tmp/hexapod_test_model.urdf

# 2. Convert to SDF.
ign sdf -p /tmp/hexapod_test_model.urdf > "$OUT_DIR/model.sdf"

# 3. Minimal model.config so Gazebo recognizes the model directory.
cat > "$OUT_DIR/model.config" << 'EOF'
<?xml version="1.0"?>
<model>
  <name>hexapod_test_model</name>
  <version>1.0</version>
  <sdf version="1.8">model.sdf</sdf>
  <description>Orbit/test copy of the hexapod model for world-embedded spawn testing.</description>
</model>
EOF

echo ""
echo "Done. Generated:"
echo "  $OUT_DIR/model.sdf"
echo "  $OUT_DIR/model.config"
echo ""
echo "Make sure IGN_GAZEBO_RESOURCE_PATH includes:"
echo "  $PKG_SHARE/models"
echo "(hexa_to_launch.py already sets this automatically)"
