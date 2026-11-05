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
- **Green cylinders** hold data. **Light grey boxes** are physical devices. **Dark grey boxes** are software we do not own (ZED SDK, Roboflow).
- **Grey dashed arrows** are data on a topic. **Amber dotted** is a request that waits for an answer (a service call or an action goal). **Green dotted** is that answer, in flows only. **Grey solid** is bringup starting a package.
- **Arrow labels** are topic, service or action names.
- Frames: `pool_link` is fixed at the deployment point. `auv_link` is the vehicle, x forward, y left, z up. In `state/pose`, z is negative below the surface. The depth topics are positive down.
- View 1 has the full legend and a short glossary.

## Drawings

The hand-drawn pictures (flat port, PID, tracker life, thrusters and more) are made by `likec4/drawings/make.mjs` with rough.js, the library behind Excalidraw.
They show in a box's side panel, in a view's description, and in the flow steps.
Every number in them comes from the code or a config file, noted next to each drawing. To change one, edit `make.mjs` and run `npm run drawings`.

## Keeping it true

When you change a topic name, a node or a launch flag, change the matching file in `likec4/model/` in the same pull request.
If you change a number that a drawing shows (gains, limits, distances), update `make.mjs` too.
