#!/bin/bash
# RViz2 capture for scripts/sim_viz.sh (inside the simulation image, glim_localization:sim):
#   sim/scripts/viz_capture.sh <out dir> <seconds> <speed-up>
# Starts Xvfb, rviz2 (sim/rviz/sim_viz.rviz) and viz_paths.py, waits for /clock (the bag playback of
# ros_localize.sh), records the screen for <seconds> and writes <out>/viz/sim_viz.webp (<speed-up>
# times faster than recorded, 5 fps, 640 px wide: about 3 MB for 60 s), sim_viz.png (the last frame)
# and logs/. Renders with llvmpipe, no GPU. Fails when rviz2 or viz_paths.py exits before the end.
set -eo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
out="${1:?usage: viz_capture.sh <out dir> <seconds> <speed-up>}"
seconds="${2:?usage: viz_capture.sh <out dir> <seconds> <speed-up>}"
speedup="${3:?usage: viz_capture.sh <out dir> <seconds> <speed-up>}"
viz="${out}/viz"
size=960x720
mkdir -p "${viz}/logs"
rm -f "${viz}/raw.mp4" "${viz}/sim_viz.webp" "${viz}/sim_viz.png"

export DISPLAY=:99
Xvfb "${DISPLAY}" -screen 0 "${size}x24" -nolisten tcp > "${viz}/logs/xvfb.log" 2>&1 &
pids=($!)
trap 'kill -TERM "${pids[@]}" 2>/dev/null || true' EXIT
sleep 1
rviz2 -d "${repo_root}/sim/rviz/sim_viz.rviz" --ros-args -p use_sim_time:=true \
  > "${viz}/logs/rviz2.log" 2>&1 &
pids+=($!)
python3 "${repo_root}/sim/scripts/viz_paths.py" "${out}/traj_gt.txt" > "${viz}/logs/viz_paths.log" 2>&1 &
pids+=($!)

echo "viz_capture: waiting for /clock"
timeout "${SIM_VIZ_WAIT:-1800}" ros2 topic echo --once /clock > /dev/null \
  || { echo "FAIL: no /clock" >&2; exit 1; }
echo "viz_capture: recording ${seconds} s"
ffmpeg -loglevel error -f x11grab -draw_mouse 0 -framerate 10 -video_size "${size}" -i "${DISPLAY}" -t "${seconds}" \
  -c:v libx264 -preset veryfast -crf 18 -pix_fmt yuv420p "${viz}/raw.mp4"
# A crashed rviz2 leaves a black screen in the recording, and a crashed viz_paths.py leaves no paths.
for i in 1 2; do
  kill -0 "${pids[i]}" 2>/dev/null || {
    log=("" rviz2 viz_paths)
    echo "FAIL: ${log[i]} exited during the capture (${viz}/logs/${log[i]}.log)" >&2
    exit 1
  }
done
# Only the 3D view: the rviz2 window (960x720 at the origin) has the menu and tool bars above it, the
# status bar below it and the handles of the hidden docks at its sides (w:h:x:y for ffmpeg crop).
crop="crop=${SIM_VIZ_CROP:-928:628:16:62}"
ffmpeg -loglevel error -i "${viz}/raw.mp4" \
  -vf "setpts=PTS/${speedup},fps=5,${crop},scale=640:-2:flags=lanczos" \
  -c:v libwebp_anim -quality "${SIM_VIZ_QUALITY:-30}" -compression_level 4 -loop 0 -an "${viz}/sim_viz.webp"
ffmpeg -loglevel error -sseof -0.5 -i "${viz}/raw.mp4" -vf "${crop}" -frames:v 1 "${viz}/sim_viz.png"
echo "viz_capture: wrote ${viz}/sim_viz.webp ($(du -k "${viz}/sim_viz.webp" | cut -f1) KiB) and sim_viz.png"
