#!/usr/bin/env bash
# bootstrap_pkg.sh — creates the so101_pose_milestone package skeleton,
# including the two intentionally-EMPTY marker files that are easy to miss.
#
# Usage:
#   cd ~/so101_ws/src          (or wherever your workspace src is)
#   bash bootstrap_pkg.sh
#
# Then copy in the 8 content files listed at the end.

set -e

PKG=so101_pose_milestone

mkdir -p $PKG/$PKG
mkdir -p $PKG/launch
mkdir -p $PKG/config
mkdir -p $PKG/resource

# The two empty files. Both are REQUIRED and both are legitimately 0 bytes:
#   resource/<pkg>  -> the ament index marker; how ROS 2 detects the package
#   <pkg>/__init__.py -> makes the inner folder an importable Python module
touch $PKG/resource/$PKG
touch $PKG/$PKG/__init__.py

echo "Skeleton created:"
find $PKG -type d | sort
echo ""
echo "Now copy these 8 content files into place:"
echo "  $PKG/package.xml"
echo "  $PKG/setup.py"
echo "  $PKG/setup.cfg"
echo "  $PKG/$PKG/targets.py"
echo "  $PKG/$PKG/pose_commander.py"
echo "  $PKG/$PKG/verify_pose.py"
echo "  $PKG/launch/pose_commander.launch.py"
echo "  $PKG/config/moveit_py_params.yaml"
echo ""
echo "Then verify with:  bash verify_pkg.sh"
