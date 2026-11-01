# AUV architecture diagrams

Interactive diagrams of the AUV software, the Unity simulator and the training pipeline.
Written in [LikeC4](https://likec4.dev), so the diagrams are plain text in `likec4/model/` and `likec4/views/`.

## View them

```bash
cd docs/architecture/likec4
npm install      # once
npm start        # opens the viewer
```

Click a box to zoom into it. The side panel has a description, the steps it follows, and a **Source** link to the code.
Check your edits with `npm run validate`. Build a static site with `npm run build`.

## What is in it

There are 22 views. You do not need all of them. Start with views 1, 2, 3, then Flow 1, then the package you work on.

| # | View | Use it to |
|---|---|---|
| 1 | The whole picture | See the three systems and the outside world |
| 2 | The AUV software | See the vehicle's loop: sense, decide, move |
| 3 | Hardware | See how each device reaches ROS |
| 4 to 7 | sensors, controls (with propulsion and teleop), vision, planner | Open a package |
| 8 to 10 | object_map, front detector, state_aggregator | Open the nodes that need it |
| 11 to 13 | Simulator, what it fakes, synthetic data and training | The sim and the detector pipeline |
| Flow 0 to 8 | Step by step | Follow one piece of data |

- Boxes with parts inside open their own view. A few nodes (navigation_server, superimposer, thrust_mapper, ObjectTracker) open a flow instead.
- Arrows between the parts of a node are hidden while the node is collapsed. They show in its flow or its own view.
- A package shows its neighbours as the real package, so you can click through to it.
- Boxes inside a node are named after real functions and classes, so you can search for them.

## Reading the diagrams

- **Blue boxes** are apps (a ROS package or a Unity subsystem). **Indigo boxes** are nodes or scripts inside them.
- **Green cylinders** hold data. **Grey boxes** are things we do not own (hardware, ZED SDK, Roboflow).
- **Arrow labels** are topic, service or action names.
- A label of `[...]` means several arrows were merged. Zoom in to see them.
- Frames: `pool_link` is fixed at the deployment point. `auv_link` is the vehicle, x forward, y left, z up. Depth is negative below the surface.

## Keeping it true

When you change a topic name, a node or a launch flag, change the matching file in `likec4/model/` in the same pull request.
