#!/bin/bash
# ECTE477 - installs package "ecte477_cr_final" into its OWN workspace ~/cr_final_ws
# and writes ~/cr_final_env.sh. Unique names = no clash with other projects on this PC.
#   bash ~/cr_final_v8/setup_cr_final.sh <ROS_DOMAIN_ID>
set -e
DOMAIN="$1"
[ -z "$DOMAIN" ] && { echo "usage: bash setup_cr_final.sh <ROS_DOMAIN_ID>"; exit 1; }
HERE="$(cd "$(dirname "$0")" && pwd)"
WS="$HOME/cr_final_ws"
PKG="$WS/src/ecte477_cr_final"
cat "$HERE/VERSION.txt"
source /opt/ros/humble/setup.bash
[ -f "$HOME/turtlebot3_ws/install/setup.bash" ] && source "$HOME/turtlebot3_ws/install/setup.bash"

echo ">>> 1/4 settings file ~/cr_final_env.sh (domain $DOMAIN)"
cat > "$HOME/cr_final_env.sh" <<ENVEOF
source /opt/ros/humble/setup.bash
[ -f \$HOME/turtlebot3_ws/install/setup.bash ] && source \$HOME/turtlebot3_ws/install/setup.bash
source \$HOME/cr_final_ws/install/setup.bash
export ROS_DOMAIN_ID=$DOMAIN
export ROS_LOCALHOST_ONLY=0
export TURTLEBOT3_MODEL=burger
ENVEOF

echo ">>> 2/4 copying package to $PKG"
KEEP=""
if [ -d "$PKG" ]; then KEEP="$PKG.prev_$(date +%H%M%S)"; mv "$PKG" "$KEEP"; touch "$KEEP/COLCON_IGNORE"; fi
mkdir -p "$WS/src"
cp -r "$HERE/ecte477_cr_final" "$WS/src/"
# keep your arena files (map + zone poses) from the previous install, if any
SRC_KEEP="$KEEP"
[ -z "$SRC_KEEP" ] && [ -d "$HOME/ros2_ws/src/construction_robot/maps" ] && \
  grep -q "zones:" "$HOME/ros2_ws/src/construction_robot/config/zones.yaml" 2>/dev/null && \
  SRC_KEEP="$HOME/ros2_ws/src/construction_robot"
if [ -n "$SRC_KEEP" ] && [ "$2" != "--fresh" ]; then
  cp "$SRC_KEEP"/maps/* "$PKG/maps/" 2>/dev/null || true
  cp "$SRC_KEEP"/config/zones.yaml "$PKG/config/zones.yaml" 2>/dev/null || true
  echo "    kept map + zones.yaml from $SRC_KEEP"
fi

echo ">>> 3/4 building"
cd "$WS"
rm -rf build/ecte477_cr_final install/ecte477_cr_final
PYTHONNOUSERSITE=1 colcon build --symlink-install --packages-select ecte477_cr_final
source "$HOME/cr_final_env.sh"

echo ">>> 4/4 checks"
echo "    ROS uses: $(ros2 pkg prefix ecte477_cr_final)"
python3 -c "import cv2; cv2.dnn.readNetFromONNX('$PKG/models/object_cnn.onnx'); print('    CNN loads OK (OpenCV', cv2.__version__, ')')"
(cd "$PKG" && python3 -m pytest -q -p no:cacheprovider test/test_vision_utils.py) || echo "!!! tests failed"
echo ">>> DONE. In EVERY new lab-PC terminal first run:  source ~/cr_final_env.sh"
