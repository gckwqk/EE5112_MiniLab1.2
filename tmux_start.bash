#!/usr/bin/env bash
# Run: bash tmux_start.bash
# Uses the original split-window / send-keys / tiled-layout structure.

SESSION_NAME=tmux_start
WORKSPACE="${MINILAB_WORKSPACE:-/home/user/EE5112_MiniLab1.2}"
ROS_SETUP=/opt/ros/humble/setup.bash

if ! tmux has-session -t "=$SESSION_NAME" 2>/dev/null; then
  for setup_file in "$ROS_SETUP" "$WORKSPACE/install/setup.bash"; do
    if [[ ! -f "$setup_file" ]]; then
      printf 'Missing setup file: %s\nBuild the workspace or set MINILAB_WORKSPACE.\n' "$setup_file" >&2
      exit 1
    fi
  done

  # Source ROS and the workspace in every pane. %q safely quotes local paths.
  printf -v PANE_SETUP 'source %q && source %q && cd %q && ' \
    "$ROS_SETUP" "$WORKSPACE/install/setup.bash" "$WORKSPACE"

  # Create a new session; explicit Bash matches the setup.bash files.
  tmux new-session -s "$SESSION_NAME" -n "$SESSION_NAME" -d \
    -x 180 -y 60 -c "$WORKSPACE" bash || exit 1

  # Mouse, history, status bar and copy/paste: same as the original script.
  tmux set -t "$SESSION_NAME" mouse on
  tmux set -t "$SESSION_NAME" history-limit 10000
  tmux set -t "$SESSION_NAME" status-bg black
  tmux set -t "$SESSION_NAME" status-fg white
  tmux setw -t "$SESSION_NAME" window-status-current-style 'bg=green,fg=black'
  tmux setw -t "$SESSION_NAME" mode-keys vi
  tmux bind-key -T copy-mode-vi 'v' send-keys -X begin-selection
  if command -v xclip >/dev/null; then
    tmux bind-key -T copy-mode-vi 'y' send-keys -X copy-pipe-and-cancel 'xclip -in -selection clipboard'
  fi

  # Visible per-pane headings. A pane option keeps headings stable even if
  # a shell changes its terminal title. Ctrl-b z zooms/unzooms a pane.
  tmux setw -t "$SESSION_NAME" pane-border-status top
  tmux setw -t "$SESSION_NAME" pane-border-format ' #{pane_index}: #{@heading} '

  # Delays only stagger startup. Wait for [READY] before giving a command.

  # ----- restart the ROS 2 daemon -----
  tmux select-pane -t "$SESSION_NAME" -T "ROS daemon"
  tmux set-option -p -t "$SESSION_NAME" @heading "ROS daemon"
  tmux send-keys -t "$SESSION_NAME" -l 'ros2 daemon stop && ros2 daemon start'
  tmux send-keys -t "$SESSION_NAME" C-m
  tmux select-layout -t "$SESSION_NAME" tiled

  # ----- start Gazebo -----
  tmux split-window -v -t "$SESSION_NAME" -c "$WORKSPACE" bash
  tmux select-pane -t "$SESSION_NAME" -T "Gazebo"
  tmux set-option -p -t "$SESSION_NAME" @heading "Gazebo"
  tmux send-keys -t "$SESSION_NAME" -l "sleep 2 && "'ros2 launch ee5112_vehicle arena.launch.py'
  tmux send-keys -t "$SESSION_NAME" C-m
  tmux select-layout -t "$SESSION_NAME" tiled

  # ----- start colour detection -----
  tmux split-window -v -t "$SESSION_NAME" -c "$WORKSPACE" bash
  tmux select-pane -t "$SESSION_NAME" -T "Colour detector"
  tmux set-option -p -t "$SESSION_NAME" @heading "Colour detector"
  tmux send-keys -t "$SESSION_NAME" -l "sleep 4 && "'ros2 launch ee5112_vehicle colour_detector.launch.py use_sim_time:=true'
  tmux send-keys -t "$SESSION_NAME" C-m
  tmux select-layout -t "$SESSION_NAME" tiled

  # ----- start localization -----
  tmux split-window -v -t "$SESSION_NAME" -c "$WORKSPACE" bash
  tmux select-pane -t "$SESSION_NAME" -T "AMCL"
  tmux set-option -p -t "$SESSION_NAME" @heading "AMCL"
  tmux send-keys -t "$SESSION_NAME" -l "sleep 6 && "'ros2 launch ee5112_vehicle amcl.launch.py'
  tmux send-keys -t "$SESSION_NAME" C-m
  tmux select-layout -t "$SESSION_NAME" tiled

  # ----- start the velocity converter -----
  tmux split-window -v -t "$SESSION_NAME" -c "$WORKSPACE" bash
  tmux select-pane -t "$SESSION_NAME" -T "Ackermann converter"
  tmux set-option -p -t "$SESSION_NAME" @heading "Ackermann converter"
  tmux send-keys -t "$SESSION_NAME" -l "sleep 6 && "'ros2 launch ee5112_vehicle cmd_vel_to_ackermann.launch.py'
  tmux send-keys -t "$SESSION_NAME" C-m
  tmux select-layout -t "$SESSION_NAME" tiled

  # ----- start the mission only (also publishes the navigation map) -----
  tmux split-window -v -t "$SESSION_NAME" -c "$WORKSPACE" bash
  tmux select-pane -t "$SESSION_NAME" -T "Task 3 mission"
  tmux set-option -p -t "$SESSION_NAME" @heading "Task 3 mission"
  tmux send-keys -t "$SESSION_NAME" -l "sleep 8 && "'ros2 launch ee5112_vehicle task3.launch.py'
  tmux send-keys -t "$SESSION_NAME" C-m
  tmux select-layout -t "$SESSION_NAME" tiled

  # ----- start Nav2 separately -----
  tmux split-window -v -t "$SESSION_NAME" -c "$WORKSPACE" bash
  tmux select-pane -t "$SESSION_NAME" -T "Nav2"
  tmux set-option -p -t "$SESSION_NAME" @heading "Nav2"
  tmux send-keys -t "$SESSION_NAME" -l "sleep 8 && "'ros2 launch ee5112_vehicle task3_nav2.launch.py'
  tmux send-keys -t "$SESSION_NAME" C-m
  tmux select-layout -t "$SESSION_NAME" tiled

  # ----- start RViz -----
  tmux split-window -v -t "$SESSION_NAME" -c "$WORKSPACE" bash
  tmux select-pane -t "$SESSION_NAME" -T "RViz"
  tmux set-option -p -t "$SESSION_NAME" @heading "RViz"
  if [[ -f "$WORKSPACE/rviz.rviz" ]]; then
    printf -v RVIZ_COMMAND 'rviz2 -d %q' "$WORKSPACE/rviz.rviz"
  else
    RVIZ_COMMAND=rviz2
  fi
  tmux send-keys -t "$SESSION_NAME" -l "sleep 10 && ${RVIZ_COMMAND}"
  tmux send-keys -t "$SESSION_NAME" C-m
  tmux select-layout -t "$SESSION_NAME" tiled

  # ----- start the typed-command interface -----
  tmux split-window -v -t "$SESSION_NAME" -c "$WORKSPACE" bash
  tmux select-pane -t "$SESSION_NAME" -T "Commands"
  tmux set-option -p -t "$SESSION_NAME" @heading "Commands"
  tmux send-keys -t "$SESSION_NAME" -l "sleep 10 && "'python3 "$(ros2 pkg prefix ee5112_vehicle)/share/ee5112_vehicle/scripts/task3_command.py"'
  tmux send-keys -t "$SESSION_NAME" C-m
  tmux select-layout -t "$SESSION_NAME" tiled
  COMMAND_PANE=$(tmux display-message -p -t "$SESSION_NAME" "#{pane_id}")

  # ----- kill session: command is prepared, Enter is NOT sent -----
  tmux split-window -v -t "$SESSION_NAME" -c "$WORKSPACE" bash
  tmux select-pane -t "$SESSION_NAME" -T 'Shutdown'
  tmux set-option -p -t "$SESSION_NAME" @heading 'Shutdown: cancel mission first, then Enter here'
  tmux send-keys -t "$SESSION_NAME" -l "gk; rk; tmux kill-session -t =$SESSION_NAME"
  tmux select-layout -t "$SESSION_NAME" tiled
  tmux select-pane -t "$COMMAND_PANE"
fi

if [[ -z "${TMUX:-}" ]]; then
  tmux attach -t "=$SESSION_NAME"
else
  tmux switch-client -t "=$SESSION_NAME"
fi
