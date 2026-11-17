# Navigation

This package provides the AUV navigation client and server, Nav2 launch files,
and Nav2 parameter files.

Launch the complete navigation stack with:

```bash
ros2 launch navigation navigation.launch.py
```

The navigation server exposes `/motion/navigate`, requests paths from Nav2,
forwards plans to the controller, and publishes setpoints to the control nodes.
