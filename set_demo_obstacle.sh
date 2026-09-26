#!/usr/bin/env bash
# Runtime physical crate presets.  Gazebo remains running; collision and LiDAR
# remain real.  The accompanying topic carries only the temporary graph block.
set -eo pipefail
cd /home/maaz-wasi/amr_ws
source /opt/ros/lyrical/setup.bash
source install/setup.bash
world=warehouse_sih_demo
name=live_demo_obstacle
preset=${1:-}
front_of_robot() {
  local ns=$1 sample json state
  sample=$(timeout 5 ros2 topic echo --once "$ns/local_status" --full-length) || {
    echo "No live status for $ns; start an executing task first." >&2; exit 1; }
  json=$(printf '%s\n' "$sample" | sed -n "s/^data: '//; s/'$//; p" | head -1)
  state=$(printf '%s' "$json" | jq -r '.state // "UNKNOWN"')
  case "$state" in WAYPOINT_TRACK|ALIGN|REACQUIRE_WAYPOINT|AVOID_FORWARD) ;; *)
    echo "$ns is $state, not executing; no obstacle was placed." >&2; exit 1;; esac
  read -r x y < <(printf '%s' "$json" | jq -r '[.pose[0],.pose[1]]|@tsv')
  read -r tx ty < <(printf '%s' "$json" | jq -r '(.remaining_route[0] // .route[-1])|@tsv')
  read -r x y < <(jq -nr -r --argjson x "$x" --argjson y "$y" --argjson tx "$tx" --argjson ty "$ty" \
    '($tx-$x) as $dx | ($ty-$y) as $dy | sqrt($dx*$dx+$dy*$dy) as $d | if $d < .1 then error("route target too close") else [($x+1.3*$dx/$d),($y+1.3*$dy/$d)]|@tsv end')
}
case "$preset" in
  CLEAR) active=false; persistent=false; zone=intersection_B; x=0; y=0 ;;
  LOCAL_AVOIDANCE) active=true; persistent=false; zone=intersection_B; x=-3.0; y=0.0 ;;
  MAIN_CORRIDOR_BLOCK) active=true; persistent=true; zone=intersection_B; x=5.0; y=0.0 ;;
  INTERSECTION_A_BLOCK) active=true; persistent=true; zone=intersection_A; x=0.0; y=0.0 ;;
  INTERSECTION_B_BLOCK) active=true; persistent=true; zone=intersection_B; x=5.0; y=0.0 ;;
  NARROW_AISLE_BLOCK) active=true; persistent=true; zone=narrow_aisle_1; x=8.0; y=4.45 ;;
  IN_FRONT_OF_ACTIVE_AMR)
    active=true; persistent=false; zone=intersection_B
    for ns in /amr_alpha /amr_bravo /amr_charlie /amr_delta /amr_echo; do
      if timeout 2 ros2 topic echo --once "$ns/local_status" >/dev/null 2>&1; then front_of_robot "$ns"; break; fi
    done
    : "${x:?No executing AMR found}" ;;
  IN_FRONT_OF_ALPHA_1) active=true; persistent=false; zone=intersection_B; front_of_robot /amr_alpha ;;
  IN_FRONT_OF_ALPHA_2) active=true; persistent=false; zone=intersection_B; front_of_robot /amr_bravo ;;
  IN_FRONT_OF_ALPHA_3) active=true; persistent=false; zone=intersection_B; front_of_robot /amr_charlie ;;
  *) echo "Usage: $0 {CLEAR|IN_FRONT_OF_ACTIVE_AMR|IN_FRONT_OF_ALPHA_1|IN_FRONT_OF_ALPHA_2|IN_FRONT_OF_ALPHA_3|MAIN_CORRIDOR_BLOCK|INTERSECTION_A_BLOCK|INTERSECTION_B_BLOCK|NARROW_AISLE_BLOCK}" >&2; exit 2 ;;
esac
# Removal is idempotent.  A fresh physical crate is then spawned at the preset.
gz service -s "/world/$world/remove" --reqtype gz.msgs.Entity --reptype gz.msgs.Boolean \
  --timeout 1000 --req "id: 0 name: \"$name\" type: MODEL" >/dev/null 2>&1 || true
if [ "$active" = true ]; then
  ros2 run ros_gz_sim create -world "$world" -name "$name" \
    -file src/amr_simulation/models/dynamic_obstacle_1/model.sdf -x "$x" -y "$y" -z 0.32 >/dev/null
fi
ros2 topic pub --once /fleet/live_obstacle std_msgs/msg/String \
  "{data: '{\"id\":\"$name\",\"preset\":\"$preset\",\"active\":$active,\"persistent\":$persistent,\"blocked_zone\":\"$zone\",\"x\":$x,\"y\":$y}'}"
