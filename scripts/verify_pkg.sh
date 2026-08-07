#!/usr/bin/env bash
# verify_pkg.sh — checks the so101_pose_milestone package is complete before
# you waste time on colcon build. Run from your workspace src/ directory.

PKG=so101_pose_milestone
fail=0

# Files that must exist AND have content
need_content=(
  "$PKG/package.xml"
  "$PKG/setup.py"
  "$PKG/setup.cfg"
  "$PKG/$PKG/targets.py"
  "$PKG/$PKG/pose_commander.py"
  "$PKG/$PKG/verify_pose.py"
  "$PKG/launch/pose_commander.launch.py"
  "$PKG/config/moveit_py_params.yaml"
)

# Files that must exist but are EXPECTED to be empty
need_empty=(
  "$PKG/resource/$PKG"
  "$PKG/$PKG/__init__.py"
)

echo "--- files needing content ---"
for f in "${need_content[@]}"; do
  if [ ! -f "$f" ]; then
    echo "  MISSING   $f"; fail=1
  elif [ ! -s "$f" ]; then
    echo "  EMPTY(!)  $f   <- downloaded but has no content"; fail=1
  else
    echo "  ok        $f  ($(wc -c < "$f") bytes)"
  fi
done

echo "--- marker files (empty is correct) ---"
for f in "${need_empty[@]}"; do
  if [ ! -f "$f" ]; then
    echo "  MISSING   $f   <- create with: touch $f"; fail=1
  else
    echo "  ok        $f"
  fi
done

echo ""
if [ $fail -eq 0 ]; then
  echo "PASS — package structure complete. Safe to colcon build."
else
  echo "INCOMPLETE — fix the items above before building."
  exit 1
fi
