// Hand-drawn explainer images for the diagrams, in the Excalidraw style (rough.js).
// Run: npm run drawings   ->  writes public/img/*.svg
// Every number drawn here comes from the code or config. The source is noted next to it.

import rough from 'roughjs'
import { mkdirSync, writeFileSync } from 'node:fs'
import { dirname, join } from 'node:path'
import { fileURLToPath } from 'node:url'

const OUT = join(dirname(fileURLToPath(import.meta.url)), '..', 'public', 'img')
mkdirSync(OUT, { recursive: true })

const C = {
  ink: '#1e1e1e', blue: '#1971c2', red: '#e03131', green: '#2f9e44', orange: '#e8590c', grey: '#868e96',
  water: '#d0ebff', air: '#fff9db', fillBlue: '#a5d8ff', fillYellow: '#ffec99', fillGreen: '#b2f2bb', fillRed: '#ffc9c9',
}
const FONT = 'Helvetica, Arial, sans-serif'

function drawing(name, w, h, seed, fn) {
  const gen = rough.generator()
  const parts = []
  const base = { seed, roughness: 1.1, bowing: 1, stroke: C.ink, strokeWidth: 1.6 }
  const add = (drawable) => {
    for (const p of gen.toPaths(drawable)) {
      parts.push(`<path d="${p.d}" stroke="${p.stroke}" stroke-width="${p.strokeWidth}" fill="${p.fill || 'none'}"/>`)
    }
  }
  const o = (extra = {}) => ({ ...base, ...extra })
  const d = {
    line: (x1, y1, x2, y2, s = {}) => add(gen.line(x1, y1, x2, y2, o(s))),
    dashed: (x1, y1, x2, y2, s = {}) =>
      parts.push(`<line x1="${x1}" y1="${y1}" x2="${x2}" y2="${y2}" stroke="${s.stroke || C.ink}" stroke-width="${s.strokeWidth || 1.5}" stroke-dasharray="6 5"/>`),
    rect: (x, y, w, h, s = {}) => add(gen.rectangle(x, y, w, h, o({ fillStyle: 'solid', ...s }))),
    circle: (cx, cy, dia, s = {}) => add(gen.circle(cx, cy, dia, o({ fillStyle: 'solid', ...s }))),
    poly: (pts, s = {}) => add(gen.polygon(pts, o({ fillStyle: 'solid', ...s }))),
    path: (pts, s = {}) => add(gen.linearPath(pts, o(s))),
    arrow: (x1, y1, x2, y2, s = {}) => {
      add(gen.line(x1, y1, x2, y2, o(s)))
      const a = Math.atan2(y2 - y1, x2 - x1), L = s.head || 12
      for (const da of [0.45, -0.45]) {
        add(gen.line(x2, y2, x2 - L * Math.cos(a + da), y2 - L * Math.sin(a + da), o(s)))
      }
    },
    text: (x, y, t, s = {}) => {
      const size = s.size || 15, anchor = s.anchor || 'start', weight = s.bold ? 'bold' : 'normal'
      const lines = String(t).split('\n')
      lines.forEach((ln, i) => parts.push(
        `<text x="${x}" y="${y + i * size * 1.25}" font-family="${FONT}" font-size="${size}" font-weight="${weight}" fill="${s.fill || C.ink}" text-anchor="${anchor}">${ln.replace(/&/g, '&amp;').replace(/</g, '&lt;')}</text>`))
    },
  }
  fn(d)
  const svg = `<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 ${w} ${h}" width="${w}" height="${h}">` +
    `<rect width="100%" height="100%" fill="#ffffff"/>${parts.join('')}</svg>`
  writeFileSync(join(OUT, `${name}.svg`), svg)
  console.log('wrote', name)
}

// ---------------------------------------------------------------------------
// 1. Frames. Source: sensors/params/sensors_frames.yaml (pool_link, auv_link, FLU), depth negative.
drawing('frames', 720, 380, 3, (d) => {
  d.text(20, 30, 'Seen from above', { bold: true, size: 17 })
  // pool_link axes
  const ox = 70, oy = 290
  d.arrow(ox, oy, ox + 140, oy, { stroke: C.blue }); d.text(ox + 146, oy + 5, 'x', { fill: C.blue, bold: true })
  d.arrow(ox, oy, ox, oy - 140, { stroke: C.blue }); d.text(ox - 6, oy - 148, 'y', { fill: C.blue, bold: true })
  d.circle(ox, oy, 10, { fill: C.blue, stroke: C.blue })
  d.text(ox - 40, oy + 32, 'pool_link: fixed. x and y start where\nthe vehicle is deployed, z = 0 at the surface', { size: 13, fill: C.blue })
  // vehicle, yawed 35 degrees
  const vx = 270, vy = 160, yaw = (35 * Math.PI) / 180
  const R = (px, py) => [vx + px * Math.cos(yaw) - py * Math.sin(yaw), vy - (px * Math.sin(yaw) + py * Math.cos(yaw))]
  d.poly([R(-50, -22), R(40, -22), R(58, 0), R(40, 22), R(-50, 22)], { fill: C.fillYellow })
  const f = R(95, 0), l = R(0, 75)
  d.arrow(vx, vy, f[0], f[1], { stroke: C.red }); d.text(f[0] + 6, f[1], 'forward (x)', { fill: C.red, size: 13 })
  d.arrow(vx, vy, l[0], l[1], { stroke: C.green }); d.text(l[0] - 20, l[1] - 8, 'left (y)', { fill: C.green, size: 13 })
  d.text(vx - 20, vy + 70, 'auv_link: moves with\nthe vehicle', { size: 13 })

  // side view
  d.text(440, 30, 'Seen from the side', { bold: true, size: 17 })
  d.rect(440, 80, 260, 240, { fill: C.water, stroke: C.blue, roughness: 0.6 })
  d.line(430, 80, 710, 80, { stroke: C.blue, strokeWidth: 2.5 })
  d.text(600, 70, 'surface: z = 0', { size: 13, fill: C.blue })
  d.arrow(470, 300, 470, 110, { stroke: C.blue }); d.text(478, 120, 'z up', { fill: C.blue, size: 13 })
  d.poly([[560, 190], [650, 190], [668, 205], [650, 220], [560, 220]], { fill: C.fillYellow })
  d.arrow(610, 82, 610, 186, { stroke: C.orange, head: 9 })
  d.text(490, 250, 'z = -1.0 m (depth 1.0 m).\nz is negative below the\nsurface. Depth is positive down.', { size: 13, fill: C.orange })
})

// ---------------------------------------------------------------------------
// 2. Flat port. Source: vision/object_detection/utils.py (Snell, n = 1.333), vision_pipeline.yaml depth_scale 1.4.
drawing('flat-port', 760, 360, 5, (d) => {
  const portX = 260, camX = 70, camY = 250, n = 1.333
  const objX = 700, objY = 70 // real object
  d.rect(20, 40, portX - 20, 270, { fill: C.air, stroke: C.grey, roughness: 0.5 })
  d.rect(portX, 40, 480, 270, { fill: C.water, stroke: C.blue, roughness: 0.5 })
  d.text(36, 66, 'air (inside the hull)', { size: 13 })
  d.text(680, 300, 'water', { size: 13, fill: C.blue })
  d.line(portX, 40, portX, 310, { strokeWidth: 4, stroke: C.ink })
  d.text(portX, 334, 'flat window (port)', { size: 13, anchor: 'middle' })
  d.rect(camX - 40, camY - 20, 60, 40, { fill: C.fillBlue }); d.text(camX - 10, camY + 5, 'camera', { size: 12, anchor: 'middle' })
  // find the point on the window where Snell's law holds (air angle bigger than water angle)
  let lo = 0, hi = camY - objY, py = 0
  for (let i = 0; i < 80; i++) {
    const m = (lo + hi) / 2, yP = camY - m
    const sa = m / Math.hypot(portX - camX, m), sw = (yP - objY) / Math.hypot(objX - portX, yP - objY)
    if (sa > n * sw) hi = m; else lo = m
    py = yP
  }
  d.line(camX + 20, camY - 6, portX, py, { stroke: C.red, strokeWidth: 2.4, roughness: 0.4 })
  d.line(portX, py, objX, objY, { stroke: C.red, strokeWidth: 2.4, roughness: 0.4 })
  // the camera assumes a straight line: continue the air ray
  const t = (objY - camY) / (py - camY), appX = camX + t * (portX - camX)
  d.dashed(portX, py, appX, objY, { stroke: C.grey })
  d.dashed(portX - 60, py, portX + 60, py, { stroke: C.grey }) // normal to the window
  d.circle(objX, objY, 26, { fill: C.fillGreen }); d.text(objX, objY + 36, 'real object', { size: 13, fill: C.green, bold: true, anchor: 'middle' })
  d.circle(appX, objY, 26, { fill: '#e9ecef', stroke: C.grey }); d.text(appX - 20, objY - 4, 'where it seems to be', { size: 13, fill: C.grey, bold: true, anchor: 'end' })
  d.text(300, 204, 'The ray bends where it crosses the window.\nFollowed straight back, it points at\nsomething closer than it really is.', { size: 14 })
  d.text(300, 262, 'Two fixes:\n1. stereo depth x 1.4, more toward the left and right edges\n2. x and y: bend the ray back with Snell (n = 1.333)', { size: 13, bold: true })
})

// ---------------------------------------------------------------------------
// 3. Down camera projection. Source: down_cam_object_detection_node.py, vision_pipeline.yaml
//    (heights: 0.67 m table items, 0.0 lane markers; pool_floor_z -2.1).
drawing('downcam-projection', 720, 360, 7, (d) => {
  d.rect(20, 20, 680, 320, { fill: C.water, stroke: C.blue, roughness: 0.5 })
  d.line(20, 320, 700, 320, { strokeWidth: 3 }); d.text(30, 312, 'pool floor (pool_floor_z = -2.1 m)', { size: 13 })
  d.rect(380, 200, 220, 18, { fill: '#e9ecef' }); d.line(400, 218, 400, 320); d.line(580, 218, 580, 320)
  d.text(540, 194, 'table', { size: 13 })
  d.poly([[180, 60], [300, 60], [318, 75], [300, 90], [180, 90]], { fill: C.fillYellow })
  d.rect(232, 90, 26, 14, { fill: C.fillBlue }); d.text(262, 104, 'down camera', { size: 12 })
  d.line(245, 104, 480, 200, { stroke: C.red, strokeWidth: 2 })
  d.circle(480, 200, 14, { fill: C.fillGreen })
  d.text(330, 132, 'pixel -> ray (Snell; bend not drawn)', { size: 13, fill: C.red })
  d.dashed(645, 200, 645, 320, { stroke: C.orange }); d.text(640, 265, '0.67 m', { size: 13, fill: C.orange, anchor: 'end' })
  d.text(40, 160, 'No depth sensor.\nThe ray stops at a plane\nwhose height we know:\ntable items 0.67 m above the floor,\nlane markers on the floor (0.0 m).', { size: 14 })
})

// ---------------------------------------------------------------------------
// 4. Track life. Source: object_tracker.cpp, vision_pipeline.yaml (min_hits 20/10, conf_to_tent_threshold 5,
//    tent_init_buffer 5, semi_persistent_conf_to_tent_threshold 450).
drawing('tracker-lifecycle', 760, 330, 9, (d) => {
  const box = (x, y, w, label, fill) => { d.rect(x, y, w, 50, { fill }); d.text(x + w / 2, y + 31, label, { anchor: 'middle', bold: true }) }
  box(20, 140, 150, 'new detection', '#e9ecef')
  box(250, 140, 150, 'TENTATIVE', C.fillYellow)
  box(530, 140, 190, 'CONFIRMED', C.fillGreen)
  box(250, 265, 150, 'deleted', C.fillRed)
  d.arrow(170, 165, 248, 165)
  d.arrow(400, 155, 528, 155, { stroke: C.green }); d.text(410, 120, '20 matches, not always in a row\n(10 for the down camera)', { size: 13, fill: C.green })
  d.arrow(528, 180, 400, 180, { stroke: C.orange }); d.text(418, 214, 'missed more than 5\nframes (450 for pipes,\nmarkers, bins)', { size: 13, fill: C.orange })
  d.arrow(325, 192, 325, 262, { stroke: C.red }); d.text(315, 230, 'tentative (new or demoted)\nand total misses over 5', { size: 13, fill: C.red, anchor: 'end' })
  d.text(560, 222, 'Published in the map.', { size: 13, bold: true })
  d.text(20, 40, 'One track per object. Static Kalman filter: nothing on the course moves.', { size: 15, bold: true })
  d.text(20, 64, 'Unique objects (one per course: gate and its panels, table and its items, octagon, board\nand its icons, bin_structure) are remembered once confirmed, and stay in the map after the track is gone.', { size: 13 })
})

// ---------------------------------------------------------------------------
// 5. Thruster layout, top view. Source: propulsion/thrust_mapper.py allocation matrix,
//    propulsion.launch.py: a 0.080619, b 0.226341, c 0.202572, d 0.228205, alpha 45 deg, disabled [4].
drawing('thrusters', 680, 660, 11, (d) => {
  const cx = 300, cy = 320, S = 700
  const P = (x, y) => [cx - y * S, cy - x * S] // forward = up, left = left
  const a = 0.080619, b = 0.226341, c = 0.202572, dd = 0.228205, k = Math.SQRT1_2
  d.poly([P(0.26, 0.15), P(0.26, -0.15), P(-0.26, -0.15), P(-0.26, 0.15)], { fill: C.fillYellow, roughness: 0.7 })
  d.arrow(cx, cy + 20, cx, cy - 60, { stroke: C.grey }); d.text(cx + 8, cy - 52, 'forward', { size: 13, fill: C.grey })
  // horizontal thrusters: position (x, y), positive-thrust direction (Fx, Fy)
  const H = [
    [1, 'back right', -dd, -c, k, -k], [4, 'front right', dd, -c, -k, -k],
    [5, 'front left', dd, c, -k, k], [8, 'back left', -dd, c, k, k],
  ]
  for (const [id, name, x, y, fx, fy] of H) {
    const [px, py] = P(x, y)
    const [ux, uy] = [-fy, -fx] // unit direction on the page
    const off = id === 4
    d.circle(px, py, 30, { fill: off ? C.fillRed : C.fillBlue })
    d.arrow(px + ux * 17, py + uy * 17, px + ux * 70, py + uy * 70, { stroke: off ? C.grey : C.blue, strokeWidth: 2.2 })
    d.text(px, py + 5, String(id), { anchor: 'middle', bold: true })
    const right = y < 0
    d.text(px + (right ? 26 : -26), py + (x > 0 ? -12 : 26), name, { anchor: right ? 'start' : 'end', size: 12 })
    if (off) {
      d.line(px - 18, py - 18, px + 18, py + 18, { stroke: C.red, strokeWidth: 3 })
      d.text(px + 26, py + 6, 'disabled at launch', { size: 12, fill: C.red, bold: true })
    }
  }
  // heave thrusters: positive thrust pushes the vehicle down (z row of the matrix is -1)
  const V = [[2, 'heave back right', -a, -b], [3, 'heave front right', a, -b], [6, 'heave front left', a, b], [7, 'heave back left', -a, b]]
  for (const [id, name, x, y] of V) {
    const [px, py] = P(x, y)
    d.circle(px, py, 30, { fill: '#e7f5ff' })
    d.line(px - 9, py - 9, px + 9, py + 9); d.line(px - 9, py + 9, px + 9, py - 9)
    d.text(px + (y < 0 ? 22 : -22), py + 5, String(id), { anchor: y < 0 ? 'start' : 'end', bold: true })
  }
  d.text(20, 34, 'Seen from above. Forward is up.', { bold: true, size: 16 })
  d.text(20, 548, 'Blue arrow: the push on the vehicle for positive thrust (horizontal thrusters, at 45 degrees).\nCrossed circle: heave thruster, pointing up and down. Positive thrust pushes the vehicle down.\nOrder in ThrusterForces and the PWM array: 1 back_right, 2 heave_back_right, 3 heave_front_right,\n4 front_right, 5 front_left, 6 heave_front_left, 7 heave_back_left, 8 back_left.\nIn the ThrusterForces message only, front_right is negated (thrust_mapper.py).', { size: 13 })
})

// ---------------------------------------------------------------------------
// 6. Pool frame vs body frame. Source: controls/src/superimposer.cpp (force_body = q_iv^-1 * force_pool).
drawing('body-frame', 700, 330, 13, (d) => {
  const ox = 150, oy = 200, yaw = (60 * Math.PI) / 180
  d.arrow(ox, oy, ox + 150, oy, { stroke: C.blue }); d.text(ox + 156, oy + 5, 'pool x', { fill: C.blue, size: 13 })
  d.arrow(ox, oy, ox, oy - 150, { stroke: C.blue }); d.text(ox - 18, oy - 158, 'pool y', { fill: C.blue, size: 13 })
  const R = (px, py) => [ox + px * Math.cos(yaw) - py * Math.sin(yaw), oy - (px * Math.sin(yaw) + py * Math.cos(yaw))]
  d.poly([R(-45, -18), R(35, -18), R(52, 0), R(35, 18), R(-45, 18)], { fill: C.fillYellow })
  const f = R(130, 0), l = R(0, 110)
  d.arrow(ox, oy, f[0], f[1], { stroke: C.red }); d.text(f[0] + 6, f[1], 'forward', { fill: C.red, size: 13 })
  d.arrow(ox, oy, l[0], l[1], { stroke: C.green }); d.text(l[0] - 40, l[1] - 6, 'left', { fill: C.green, size: 13 })
  d.arrow(ox, oy + 6, ox + 120, oy + 6, { stroke: C.orange, strokeWidth: 3.2 })
  d.text(ox + 30, oy + 32, 'push: +x in the pool frame', { fill: C.orange, size: 13, bold: true })
  d.text(360, 80, 'Controller says: push along pool x.', { size: 15, bold: true })
  d.text(360, 110, 'The IMU says the vehicle is turned 60 degrees.\nIn its own frame that same push is\n0.5 forward and 0.87 to the right.', { size: 14 })
  d.text(360, 200, 'superimposer does this turn with\nthe IMU orientation, so the thrusters\nget the push in their own frame.', { size: 14 })
})

// ---------------------------------------------------------------------------
// 7. Gate approach. Source: planner/missions/robosub2026/gate_task.py (aims at the role panel),
//    behaviour_tree_params.yaml: approach_distance 2.0, pass_distance 0.5, desired_role.
drawing('gate-approach', 760, 390, 15, (d) => {
  const gx = 470 // gate line, vertical; the vehicle moves left to right
  d.line(gx, 70, gx, 330, { strokeWidth: 3 })
  d.rect(gx - 8, 80, 16, 110, { fill: C.fillGreen })
  d.rect(gx - 8, 210, 16, 110, { fill: '#e9ecef' }); d.text(gx + 16, 270, 'other panel', { size: 13, fill: C.grey })
  d.text(gx, 62, 'gate', { size: 14, bold: true, anchor: 'middle' })
  d.text(gx + 16, 100, 'chosen panel', { size: 13, fill: C.green, bold: true })
  d.text(gx + 16, 116, '(desired_role)', { size: 12, fill: C.green })
  const S = 110, panelY = 150
  const ax = gx - 2.0 * S, px = gx + 0.5 * S
  d.circle(ax, panelY, 16, { fill: C.fillYellow }); d.text(ax, panelY - 18, 'approach point', { size: 13, bold: true, anchor: 'middle' })
  d.circle(px, panelY, 16, { fill: C.fillYellow }); d.text(px + 14, panelY + 5, 'pass point', { size: 13, bold: true })
  d.arrow(60, 320, ax - 10, panelY + 10, { stroke: C.blue }); d.text(120, 300, 'goal 1', { size: 13, fill: C.blue })
  d.arrow(ax + 12, panelY, px - 12, panelY, { stroke: C.blue }); d.text(ax + 60, panelY + 20, 'goal 2', { size: 13, fill: C.blue })
  d.dashed(ax, panelY + 34, gx, panelY + 34, { stroke: C.orange }); d.text((ax + gx) / 2, panelY + 54, 'approach_distance 2.0 m', { size: 13, fill: C.orange, anchor: 'middle' })
  d.dashed(gx, panelY + 34, px, panelY + 34, { stroke: C.orange }); d.text(px + 6, panelY + 54, 'pass_distance 0.5 m', { size: 13, fill: C.orange })
  d.text(20, 34, 'Aim at the chosen panel, not the middle of the gate.', { size: 16, bold: true })
  d.text(20, 372, 'Each half of the gate shows a different picture (a role panel). desired_role picks one before the run.', { size: 13 })
})

// ---------------------------------------------------------------------------
// 8. PID, with the depth controller as the example. Source: controls/scripts/depth_controller.py,
//    controls/controls/pid.py, Controller_params_real.yaml (KP 30, KD 50, KI 0.15, threshold 0.3, net_buoyancy 3.5, 10 Hz).
drawing('pid-loop', 780, 400, 17, (d) => {
  d.text(20, 34, 'One controller, one axis. Example: depth_controller.', { size: 16, bold: true })
  d.rect(20, 60, 250, 310, { fill: C.water, stroke: C.blue, roughness: 0.5 })
  d.line(10, 60, 280, 60, { stroke: C.blue, strokeWidth: 2.5 }); d.text(150, 52, 'surface', { size: 12, fill: C.blue })
  d.dashed(30, 250, 260, 250, { stroke: C.green }); d.text(36, 268, 'target: depth 1.0 m (z = -1.0)', { size: 12, fill: C.green, bold: true })
  d.poly([[90, 145], [180, 145], [198, 160], [180, 175], [90, 175]], { fill: C.fillYellow })
  d.text(36, 135, 'now: depth 0.6 m (z = -0.6)', { size: 12, bold: true })
  d.arrow(145, 180, 145, 240, { stroke: C.orange, strokeWidth: 2.4 }); d.text(152, 215, 'error -0.4 m', { size: 12, fill: C.orange, bold: true })
  d.text(36, 340, 'The math uses z (up is +),\nso the error is -1.0 - (-0.6).', { size: 12 })

  const x = 300
  d.text(x, 80, 'effort = P + I + D + feed-forward', { size: 15, bold: true })
  d.text(x, 115, 'P   30 x error = 30 x -0.4 = -12 N. Push down, harder when further away.', { size: 13 })
  d.text(x, 150, 'D   -50 x how fast z is changing (smoothed). Brakes as it closes in,\n     so it does not overshoot.', { size: 13 })
  d.text(x, 203, 'I    0.15 x error added up over time. Fixes a small steady miss.\n     Only counts within 0.3 m of the target, and resets when the target changes.', { size: 13 })
  d.text(x, 256, 'FF  -3.5 N, always. The vehicle floats up with 3.5 N (net_buoyancy),\n     so this cancels it before the PID even starts.', { size: 13 })
  d.text(x, 312, 'Out: /controls/depth_effort, a force in the pool frame, 10 times a second.', { size: 13, bold: true })
  d.text(x, 340, 'x and y work the same way with the DVL (KP 10, KD 7.5, KI 0.5).\nattitude_controller does it on orientation, 25 times a second.', { size: 13, fill: C.grey })
})

// ---------------------------------------------------------------------------
// 9. When a navigate goal is done. Source: controls/scripts/navigation_server.py (feedback_rate_hz 10,
//    time_in_tolerance, hold_time, timeout), behaviour_tree_params.yaml (0.3 m, 5 s, 60 s).
drawing('nav-hold', 780, 380, 19, (d) => {
  d.text(20, 34, 'When does a navigate goal finish?', { size: 16, bold: true })
  const ox = 70, oy = 300, W = 620
  d.arrow(ox, oy, ox + W, oy); d.text(ox + W - 10, oy + 22, 'time', { size: 13, anchor: 'end' })
  d.arrow(ox, oy, ox, 70); d.text(ox - 8, 66, 'distance to target', { size: 13 })
  const tolY = 230
  d.rect(ox + 2, tolY, W - 4, oy - tolY - 2, { fill: C.fillGreen, stroke: 'none', roughness: 0.2 })
  d.dashed(ox, tolY, ox + W, tolY, { stroke: C.green }); d.text(ox + W - 6, tolY - 8, 'position_tolerance (e.g. 0.3 m)', { size: 12, fill: C.green, anchor: 'end', bold: true })
  const pts = [[ox, 90], [ox + 60, 120], [ox + 120, 170], [ox + 170, 215], [ox + 200, 240], [ox + 225, 248], [ox + 250, 236], [ox + 270, 222], [ox + 295, 226], [ox + 320, 250], [ox + 360, 262], [ox + 420, 266], [ox + 480, 268], [ox + 520, 269]]
  d.path(pts, { stroke: C.blue, strokeWidth: 2.4 })
  d.text(ox + 205, 275, 'inside...', { size: 12, fill: C.blue })
  d.text(ox + 245, 205, 'out again:\nclock resets to 0', { size: 12, fill: C.red, bold: true })
  d.line(ox + 320, oy + 8, ox + 320, oy + 20, { stroke: C.orange }); d.line(ox + 520, oy + 8, ox + 520, oy + 20, { stroke: C.orange })
  d.line(ox + 320, oy + 14, ox + 520, oy + 14, { stroke: C.orange }); d.text(ox + 420, oy + 34, 'stays inside for hold_time (e.g. 5 s)', { size: 12, fill: C.orange, anchor: 'middle', bold: true })
  d.circle(ox + 520, 269, 14, { fill: C.fillGreen }); d.text(ox + 530, 255, 'success', { size: 13, fill: C.green, bold: true })
  d.text(20, 360, 'Checked 10 times a second. The angle must also be inside angular_tolerance. If timeout (e.g. 60 s) runs out first: failure.', { size: 12 })
})

// ---------------------------------------------------------------------------
// 10. How state/pose is built. Source: sensors/src/State_aggregator.cpp (publish_state, 50 Hz,
//     z = -current_depth_, VIO off by default).
drawing('pose-fusion', 780, 340, 21, (d) => {
  d.text(20, 34, 'state/pose is put together, not filtered. Each part comes from one sensor.', { size: 16, bold: true })
  const src = (y, title, sub, fill) => { d.rect(20, y, 220, 58, { fill }); d.text(32, y + 24, title, { size: 14, bold: true }); d.text(32, y + 44, sub, { size: 12 }) }
  src(60, 'IMU', 'auv_frame/imu', C.fillBlue)
  src(140, 'Depth sensor', 'auv_frame/depth: 1.0 (positive down)', C.fillYellow)
  src(220, 'DVL', 'auv_frame/dvl/position', '#d3f9d8')
  d.rect(510, 120, 250, 100, { fill: '#f3f0ff' })
  d.text(522, 146, 'state/pose, 50 Hz', { size: 15, bold: true })
  d.text(522, 170, 'orientation   from the IMU', { size: 12 })
  d.text(522, 188, 'x, y             from the DVL', { size: 12 })
  d.text(522, 206, 'z                  = -depth = -1.0', { size: 12 })
  d.arrow(240, 89, 506, 150, { stroke: C.blue }); d.text(320, 106, 'orientation', { size: 12, fill: C.blue })
  d.arrow(240, 169, 506, 175, { stroke: C.orange }); d.text(300, 162, 'z = -depth', { size: 12, fill: C.orange, bold: true })
  d.arrow(240, 249, 506, 200, { stroke: C.green }); d.text(320, 252, 'x, y', { size: 12, fill: C.green })
  d.text(20, 312, 'Here depth (positive down) becomes z in state/pose. navigation_server turns its z target back into depth_setpoint,\nand depth_controller flips both to z for its math. VIO can replace orientation or x, y, but it is off.', { size: 12 })
})

// ---------------------------------------------------------------------------
// 11. The mission tree. Source: planner/missions/mission_sequence.py (MissionSpawner, startup sequence,
//     choice 19 full run), behaviour_tree_params.yaml (tick_rate 3, startup_delay 30, transitions disabled).
drawing('mission-tree', 800, 440, 23, (d) => {
  const node = (x, y, w, t, fill = '#ffffff', size = 12) => { d.rect(x, y, w, 40, { fill }); d.text(x + w / 2, y + 25, t, { size, anchor: 'middle', bold: true }) }
  d.text(20, 30, 'The tree is ticked 3 times a second. A sequence runs its children left to right:', { size: 15, bold: true })
  d.text(20, 50, 'the next one starts only when the one before it succeeds.', { size: 15, bold: true })
  node(300, 70, 200, 'Mission Runner', '#f3f0ff', 13)
  node(60, 160, 260, '1. wait for a number on /mission_selector', C.fillYellow)
  node(470, 160, 200, '2. active mission', '#f3f0ff')
  d.line(400, 110, 190, 160); d.line(400, 110, 570, 160)
  node(300, 250, 230, 'startup: wait 30 s', C.fillBlue)
  d.text(415, 305, 'then enable the four controllers', { size: 11, anchor: 'middle' })
  node(580, 250, 200, 'chosen mission (19 = full run)', C.fillGreen)
  d.line(570, 200, 415, 250); d.line(570, 200, 680, 250)
  const tasks = ['Gate', 'Slalom', 'Bins', 'Torpedo', 'Octagon', 'Home']
  tasks.forEach((t, i) => {
    const x = 210 + i * 96
    node(x, 350, 86, t, '#ffffff', 12)
    d.line(680, 290, x + 43, 350, { stroke: C.grey })
  })
  d.text(20, 420, 'Numbers 1 to 18 load one task or test. 19 loads the full run drawn here. In it, a failed task counts as done, so the run\nmoves on. Bins has a 120 s limit. The run is recorded to a rosbag. Transition moves between tasks are off in the config.', { size: 12 })
})

// ---------------------------------------------------------------------------
// 12. Camera frame to pool frame. Source: vision/object_map/object_map.cpp (auv_pose in the detection frame,
//     camera-to-AUV from TF, covariance rotated the same way).
drawing('to-pool-frame', 780, 360, 25, (d) => {
  d.text(20, 34, 'object_map: from "3 m in front of the camera" to a spot in the pool', { size: 16, bold: true })
  const ox = 60, oy = 310, S = 45
  d.arrow(ox, oy, ox + 300, oy, { stroke: C.blue }); d.text(ox + 306, oy + 5, 'pool x', { size: 12, fill: C.blue })
  d.arrow(ox, oy, ox, 60, { stroke: C.blue }); d.text(ox + 6, 62, 'pool y', { size: 12, fill: C.blue })
  const vx = ox + 4 * S, vy = oy - 2 * S
  d.poly([[vx - 14, vy + 30], [vx - 14, vy - 20], [vx, vy - 34], [vx + 14, vy - 20], [vx + 14, vy + 30]], { fill: C.fillYellow })
  d.text(vx + 22, vy + 28, 'AUV at (4, 2),\nfacing pool y', { size: 12 })
  d.dashed(vx, vy - 34, vx, oy - 5 * S, { stroke: C.red })
  d.text(vx + 10, vy - 70, '3 m ahead', { size: 12, fill: C.red, bold: true })
  d.circle(vx, oy - 5 * S, 22, { fill: C.fillGreen }); d.text(vx + 18, oy - 5 * S - 12, 'gate at (4, 5)', { size: 12, fill: C.green, bold: true })
  const x = 430
  d.text(x, 90, '1. The detection says: 3 m ahead of the camera.', { size: 13 })
  d.text(x, 125, '2. The AUV pose saved with that frame says\n    where the vehicle was and which way it faced.', { size: 13 })
  d.text(x, 178, '3. TF says where the camera sits on the hull.', { size: 13 })
  d.text(x, 213, '4. Chain them: pool <- AUV <- camera.', { size: 13, bold: true })
  d.text(x, 248, 'The uncertainty is turned the same way,\nso "unsure about distance" stays along\nthe line of sight.', { size: 13 })
  d.text(x, 320, 'No pose with the frame, no position.\nThat is why the pose travels with the detections.', { size: 12, fill: C.grey })
})

// ---------------------------------------------------------------------------
// 13. The sim loop. Source: Flow 7 sources (thrust_mapper propulsion/forces, ros_tcp_endpoint 10000,
//     Thrusters.cs, Sensors/*, SensorDelayBuffer.cs, ZED2iSimSender port 30000).
drawing('sim-loop', 800, 400, 27, (d) => {
  d.text(20, 32, 'The sim replaces the vehicle and the pool. The ROS side does not know.', { size: 16, bold: true })
  d.rect(20, 60, 300, 290, { fill: '#f3f0ff', stroke: C.grey, roughness: 0.5 }); d.text(36, 84, 'ROS 2 (same code as the vehicle)', { size: 13, bold: true })
  d.rect(470, 60, 310, 290, { fill: C.water, stroke: C.blue, roughness: 0.5 }); d.text(486, 84, 'Unity', { size: 13, bold: true, fill: C.blue })
  const box = (x, y, w, t, fill) => { d.rect(x, y, w, 44, { fill }); d.text(x + w / 2, y + 27, t, { size: 12, anchor: 'middle', bold: true }) }
  box(50, 110, 240, 'thrust_mapper', '#ffffff')
  box(50, 290, 240, 'sensors, vision', '#ffffff')
  box(500, 110, 250, 'thrusters push the rigidbody', C.fillYellow)
  box(500, 200, 250, '+ buoyancy, drag: it moves', C.fillYellow)
  box(500, 290, 250, 'sensors sample it, add noise', C.fillYellow)
  d.arrow(290, 132, 498, 132, { stroke: C.orange }); d.text(394, 124, 'eight forces (N)', { size: 12, anchor: 'middle', fill: C.orange, bold: true })
  d.text(394, 152, 'TCP, port 10000', { size: 11, anchor: 'middle', fill: C.grey })
  d.arrow(625, 154, 625, 198); d.arrow(625, 244, 625, 288)
  d.arrow(498, 312, 292, 312, { stroke: C.blue }); d.text(394, 304, 'imu, dvl, depth, down camera', { size: 12, anchor: 'middle', fill: C.blue, bold: true })
  d.text(394, 332, 'imu and dvl arrive late on purpose', { size: 11, anchor: 'middle', fill: C.grey })
  d.arrow(498, 366, 292, 366, { stroke: C.green }); d.text(394, 388, 'front camera: through the ZED SDK, port 30000', { size: 11, anchor: 'middle', fill: C.green })
  d.text(36, 200, 'On the vehicle the thrusters get PWM.\nUnity needs forces, so thrust_mapper\nalso publishes propulsion/forces.', { size: 12, fill: C.grey })
})

// ---------------------------------------------------------------------------
// 14. Synthetic data. Source: SynthDataGen (BatchRunner, randomizers, labelers), YOLO converter,
//     vision/model_pipeline (split 70/20/10, RF-DETR, ONNX, trtexec on the Jetson).
drawing('synth-data', 820, 320, 29, (d) => {
  d.text(20, 32, 'Unity knows where every object is, so every image comes labelled for free.', { size: 16, bold: true })
  const steps = [
    ['Randomize', 'objects, camera,\nwater colour, textures', C.fillYellow],
    ['Render', 'HDRP, 672x376,\nthe ZED resolution', C.water],
    ['Label', 'boxes, masks,\nhow hidden it is', C.fillGreen],
    ['Convert', 'SOLO to YOLO,\ndrop hidden objects', '#e9ecef'],
    ['Train', 'RF-DETR, export\nto ONNX, TensorRT\non the Jetson', '#f3f0ff'],
  ]
  steps.forEach(([t, sub, fill], i) => {
    const x = 20 + i * 160
    d.rect(x, 70, 140, 120, { fill })
    d.text(x + 70, 100, t, { size: 15, bold: true, anchor: 'middle' })
    d.text(x + 70, 130, sub, { size: 11, anchor: 'middle' })
    if (i < steps.length - 1) d.arrow(x + 142, 130, x + 158, 130)
  })
  d.text(20, 230, 'Each batch changes the ranges: clear or murky water, near or far, front or down camera.', { size: 13 })
  d.text(20, 255, 'The look changes every frame, so the model learns the object, not one pool.', { size: 13 })
  d.text(20, 290, 'Then a smaller set of real images, pre-labelled by this model and fixed by people in Roboflow, fine-tunes it.', { size: 12, fill: C.grey })
})
