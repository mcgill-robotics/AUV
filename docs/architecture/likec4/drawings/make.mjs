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
  d.text(500, 250, 'z = -1.0 m.\nDepth is negative\nbelow the surface.', { size: 13, fill: C.orange })
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
  d.text(300, 210, 'The ray bends where it crosses the window.\nFollowed straight back, it points at\nsomething closer than it really is.', { size: 14 })
  d.text(300, 275, 'Fix: scale depth back up, about x1.4\n(a bit more toward the edges of the image).', { size: 14, bold: true })
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
  d.text(560, 216, 'Published in the map.', { size: 13, bold: true })
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
