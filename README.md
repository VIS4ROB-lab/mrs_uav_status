# MRS UAV Status

Please follow to the [documentation page](https://ctu-mrs.github.io/docs/features/status_tui/).

## Python GUI (tmux replacement)

A simple Tkinter GUI is now available to avoid the tmux-based interface. It listens to the same `uav_status` and `uav_status_short` topics and renders a compact dashboard per UAV.

Run it with:

```bash
ros2 run mrs_uav_status status_gui.py -- --uavs uav1 uav2
```

Notes:
- The GUI defaults to `$UAV_NAME` or `uav1` if no `--uavs` are provided.
- Ensure `python3-tk` is installed on the target machine so Tkinter is available.
- Topics are resolved as `/<uav>/<uav_status_acquisition/...>` by default; override with `--status-topic` or `--status-short-topic` if needed.
- A remote control panel is included per UAV: `w/k/Up` pitch forward, `s/j/Down` pitch back, `a/h/Left` roll left, `d/l/Right` roll right, `r` thrust up, `f` thrust down, `q` yaw +, `e` yaw -, `Hover` calls the hover service, `Turbo (T)` swaps constraints (default `--turbo-constraints fast`), and `Global frame (G)` toggles between odom-frame moves and body-frame moves (falls back to odom if TF is missing).
