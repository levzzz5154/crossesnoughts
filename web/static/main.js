/* YDLidar X3 scene viz — three.js scene graph, latest-wins WS client. */
import * as THREE from "three";
import { OrbitControls } from "three/addons/OrbitControls.js";

const hud = {
  status: document.getElementById("hud-status"),
  seq: document.getElementById("hud-seq"),
  freq: document.getElementById("hud-freq"),
  track: document.getElementById("hud-track"),
  cell: document.getElementById("hud-cell"),
};
const errorBox = document.getElementById("error");

let renderer;
try {
  renderer = new THREE.WebGLRenderer({ antialias: true });
} catch (e) {
  errorBox.hidden = false;
  errorBox.textContent = "WebGL unavailable: " + e.message;
  hud.status.textContent = "error";
  throw e;
}
renderer.setSize(window.innerWidth, window.innerHeight);
renderer.setPixelRatio(window.devicePixelRatio);
document.body.appendChild(renderer.domElement);

const scene = new THREE.Scene();
scene.background = new THREE.Color(0x101418);

const camera = new THREE.PerspectiveCamera(60, window.innerWidth / window.innerHeight, 0.01, 50);
const controls = new OrbitControls(camera, renderer.domElement);
controls.enableDamping = true;
controls.dampingFactor = 0.08;

// lights
scene.add(new THREE.HemisphereLight(0xffffff, 0x334455, 0.9));
const dir = new THREE.DirectionalLight(0xffffff, 1.2);
dir.position.set(3, 6, 4);
scene.add(dir);

// --- scene graph (built after hello config arrives) ---
let S = 2.0;
let boxGroup, boardGroup, lidarMarker, pointCloud, trackMarker, truePoseMarker;
let gridGroup, cellHighlight, clickPlane, clickMarker;
let positions = null;

function buildScene(cfg) {
  S = cfg.board_size;
  scene.clear();
  scene.add(new THREE.HemisphereLight(0xffffff, 0x334455, 0.9));
  const dir2 = new THREE.DirectionalLight(0xffffff, 1.2);
  dir2.position.set(3, 6, 4);
  scene.add(dir2);

  // floor
  const floor = new THREE.Mesh(
    new THREE.PlaneGeometry(S + 2, S + 2),
    new THREE.MeshStandardMaterial({ color: 0x222a33, side: THREE.DoubleSide })
  );
  floor.rotation.x = -Math.PI / 2;
  floor.position.y = -0.02;
  scene.add(floor);
  const grid = new THREE.GridHelper(S + 2, 10, 0x3a4a5a, 0x2a3542);
  grid.position.y = -0.01;
  scene.add(grid);

  // box walls (4 semi-transparent)
  boxGroup = new THREE.Group();
  const wallMat = new THREE.MeshStandardMaterial({
    color: 0x88aacc, opacity: 0.25, transparent: true, side: THREE.DoubleSide,
  });
  const h = cfg.box_h;
  const half = S / 2 + 1; // walls at x/z = +-(S/2+1)
  const wallGeo = new THREE.PlaneGeometry(S + 2, h);
  for (const [x, z, ry] of [[half, 0, Math.PI / 2], [-half, 0, Math.PI / 2], [0, half, 0], [0, -half, 0]]) {
    const m = new THREE.Mesh(wallGeo, wallMat);
    m.position.set(x, h / 2, z);
    m.rotation.y = ry;
    boxGroup.add(m);
  }
  scene.add(boxGroup);

  // board
  boardGroup = new THREE.Group();
  const board = new THREE.Mesh(
    new THREE.BoxGeometry(S, 0.02, S),
    new THREE.MeshStandardMaterial({ color: 0x3a4a5a })
  );
  board.position.y = -0.01;
  boardGroup.add(board);
  const edges = new THREE.LineSegments(
    new THREE.EdgesGeometry(new THREE.BoxGeometry(S, 0.021, S)),
    new THREE.LineBasicMaterial({ color: 0x88aacc })
  );
  scene.add(boardGroup);

  // 3x3 crosses-noughts grid overlay (board coords: cell i = x in [i*S/3,(i+1)*S/3])
  gridGroup = new THREE.Group();
  const lineMat = new THREE.LineBasicMaterial({ color: 0xffd27a, transparent: true, opacity: 0.85 });
  const step = S / 3;
  for (let i = 1; i < 3; i++) {
    const vx = new THREE.BufferGeometry().setFromPoints([
      new THREE.Vector3(-S / 2 + i * step, 0.012, -S / 2),
      new THREE.Vector3(-S / 2 + i * step, 0.012, S / 2),
    ]);
    const vz = new THREE.BufferGeometry().setFromPoints([
      new THREE.Vector3(-S / 2, 0.012, -S / 2 + i * step),
      new THREE.Vector3(S / 2, 0.012, -S / 2 + i * step),
    ]);
    gridGroup.add(new THREE.Line(vx, lineMat));
    gridGroup.add(new THREE.Line(vz, lineMat));
  }
  scene.add(gridGroup);

  // nearest-cell highlight (invisible until a frame arrives)
  cellHighlight = new THREE.Mesh(
    new THREE.PlaneGeometry(step, step),
    new THREE.MeshBasicMaterial({ color: 0xffd27a, transparent: true, opacity: 0.25, side: THREE.DoubleSide })
  );
  cellHighlight.rotation.x = -Math.PI / 2;
  cellHighlight.position.y = 0.015;
  cellHighlight.visible = false;
  scene.add(cellHighlight);

  // invisible board-plane for click/drag raycasting (only the board)
  clickPlane = new THREE.Mesh(
    new THREE.PlaneGeometry(S, S),
    new THREE.MeshBasicMaterial({ visible: false })
  );
  clickPlane.rotation.x = -Math.PI / 2;
  clickPlane.position.y = 0.01;
  scene.add(clickPlane);

  // marker for the clicked/placed position
  clickMarker = new THREE.Mesh(
    new THREE.SphereGeometry(0.035, 16, 16),
    new THREE.MeshStandardMaterial({ color: 0xffff44 })
  );
  clickMarker.position.y = 0.05;
  clickMarker.visible = false;
  scene.add(clickMarker);

  // lidar marker at board front-right corner -> world (-S/2, 0, -S/2)
  lidarMarker = new THREE.Group();
  const cyl = new THREE.Mesh(
    new THREE.CylinderGeometry(0.03, 0.03, 0.06, 16),
    new THREE.MeshStandardMaterial({ color: 0xff8800 })
  );
  cyl.position.y = 0.03;
  lidarMarker.add(cyl);
  const arrow = new THREE.ArrowHelper(new THREE.Vector3(0, 0, 1), new THREE.Vector3(0, 0.07, 0), 0.3, 0xff8800);
  lidarMarker.add(arrow);
  lidarMarker.position.set(-S / 2, 0, -S / 2);
  scene.add(lidarMarker);

  // point cloud: preallocated 300 points, setDrawRange per frame
  positions = new Float32Array(300 * 3);
  const pgeo = new THREE.BufferGeometry();
  pgeo.setAttribute("position", new THREE.BufferAttribute(positions, 3));
  pgeo.setDrawRange(0, 0);
  pointCloud = new THREE.Points(
    pgeo,
    new THREE.PointsMaterial({ color: 0x00ff88, size: 0.012 })
  );
  scene.add(pointCloud);

  // track marker
  trackMarker = new THREE.Mesh(
    new THREE.SphereGeometry(0.04, 16, 16),
    new THREE.MeshStandardMaterial({ color: 0xff2222 })
  );
  trackMarker.visible = false;
  scene.add(trackMarker);

  // true pose marker (sim only)
  truePoseMarker = new THREE.Mesh(
    new THREE.SphereGeometry(0.03, 16, 16),
    new THREE.MeshStandardMaterial({ color: 0x22ff22 })
  );
  truePoseMarker.visible = false;
  scene.add(truePoseMarker);

  camera.position.set(1.0 * S, 0.9 * S, 1.0 * S);
  camera.lookAt(0, 0.1, 0);
  controls.target.set(0, 0.1, 0);
  controls.update();
}

let pending = null; // latest-wins: only the newest frame is applied per rAF
let lastSeq = 0;

function applyFrame(f) {
  lastSeq = f.seq;
  hud.seq.textContent = "seq " + f.seq;
  hud.freq.textContent = "freq " + (f.scan_freq || 0).toFixed(1) + " Hz";
  if (positions) {
    const n = f.points.length;
    for (let i = 0; i < n; i++) {
      // B->W translation only: x_W = x_B - S/2, z_W = y_B - S/2, y = 0.02
      positions[i * 3] = f.points[i].x - S / 2;
      positions[i * 3 + 1] = 0.02;
      positions[i * 3 + 2] = f.points[i].y - S / 2;
    }
    if (n < 300) {
      for (let i = n; i < 300; i++) positions[i * 3 + 1] = -100; // hide
    }
    pointCloud.geometry.setDrawRange(0, n);
    pointCloud.geometry.attributes.position.needsUpdate = true;
  }
  if (f.tracked) {
    trackMarker.visible = true;
    trackMarker.position.set(f.tracked.x - S / 2, 0.05, f.tracked.y - S / 2);
    hud.track.textContent =
      "track (" + f.tracked.x.toFixed(2) + ", " + f.tracked.y.toFixed(2) +
      ") conf " + f.tracked.conf.toFixed(2) + " age " + f.tracked.age;
    // nearest cell to the tracked output: floor(x*3/S), floor(y*3/S), clamped
    const cx = Math.min(2, Math.max(0, Math.floor(f.tracked.x * 3 / S)));
    const cy = Math.min(2, Math.max(0, Math.floor(f.tracked.y * 3 / S)));
    hud.cell.textContent = "cell (" + cx + ", " + cy + ")";
    cellHighlight.visible = true;
    cellHighlight.position.set(-S / 2 + (cx + 0.5) * S / 3, 0.015, -S / 2 + (cy + 0.5) * S / 3);
  } else {
    trackMarker.visible = false;
    cellHighlight.visible = false;
    hud.track.textContent = "track —";
    hud.cell.textContent = "cell —";
  }
  if (f.true_pose) {
    truePoseMarker.visible = true;
    truePoseMarker.position.set(f.true_pose.x - S / 2, 0.05, f.true_pose.y - S / 2);
  } else {
    truePoseMarker.visible = false;
  }
}

function animate() {
  requestAnimationFrame(animate);
  if (pending) {
    const f = pending;
    pending = null;
    applyFrame(f);
  }
  controls.update();
  renderer.render(scene, camera);
}
animate();

// --- WS client with reconnect + 1 s backoff ---
let ws = null;
let retryTimer = null;

function connect() {
  hud.status.textContent = "connecting…";
  const proto = location.protocol === "https:" ? "wss" : "ws";
  ws = new WebSocket(proto + "://" + location.host + "/ws");
  ws.onopen = () => { hud.status.textContent = "connected"; };
  ws.onmessage = (ev) => {
    let msg;
    try { msg = JSON.parse(ev.data); } catch { return; }
    if (msg.type === "hello") {
      buildScene(msg.config);
    } else if (msg.type === "scene") {
      // geometry already drawn from hello; ignored
    } else if (msg.type === "frame") {
      if (msg.seq > lastSeq) pending = msg;
    }
  };
  ws.onclose = () => {
    hud.status.textContent = "disconnected — retrying";
    retryTimer = setTimeout(connect, 1000);
  };
  ws.onerror = () => { try { ws.close(); } catch {} };
}
connect();

// --- click/drag to place the object on the board ---------------------------
// Left-drag orbits (OrbitControls); left-click (no drag) or right-drag places
// the object at the pointer position. The position is sent to the server via
// {"type":"set_object","x","y"} (board coords) — the sim moves the object
// there on the next scan boundary.
const raycaster = new THREE.Raycaster();
const ndc = new THREE.Vector2();
let dragging = false;
let moved = false;

function boardPoint(event) {
  const rect = renderer.domElement.getBoundingClientRect();
  ndc.x = ((event.clientX - rect.left) / rect.width) * 2 - 1;
  ndc.y = -((event.clientY - rect.top) / rect.height) * 2 + 1;
  raycaster.setFromCamera(ndc, camera);
  const hit = raycaster.intersectObject(clickPlane, false);
  if (hit.length === 0) return null;
  // world -> board: x_B = x_W + S/2, y_B = z_W + S/2
  return { x: hit[0].point.x + S / 2, y: hit[0].point.z + S / 2 };
}

function sendObject(x, y) {
  if (ws && ws.readyState === WebSocket.OPEN) {
    ws.send(JSON.stringify({ type: "set_object", x, y }));
  }
  clickMarker.visible = true;
  clickMarker.position.set(x - S / 2, 0.05, y - S / 2);
  hud.status.textContent = "click (" + x.toFixed(2) + ", " + y.toFixed(2) + ")";
}
// debug hook for automated verification
window.__viz = { boardPoint, sendObject, getClickPlane: () => clickPlane, getWs: () => ws };

renderer.domElement.addEventListener("pointerdown", (e) => {
  if (e.button !== 0 || !clickPlane) return;
  dragging = true;
  moved = false;
});
renderer.domElement.addEventListener("pointermove", (e) => {
  if (!dragging || !clickPlane) return;
  moved = true;
  const p = boardPoint(e);
  if (p) sendObject(p.x, p.y);
});
renderer.domElement.addEventListener("pointerup", (e) => {
  if (!dragging) return;
  dragging = false;
  if (!moved && e.button === 0) {
    // plain click (no drag): place the object
    const p = boardPoint(e);
    if (p) sendObject(p.x, p.y);
  }
});
// right-drag also places (OrbitControls uses left-drag by default)
renderer.domElement.addEventListener("contextmenu", (e) => e.preventDefault());
renderer.domElement.addEventListener("pointerdown", (e) => {
  if (e.button !== 2 || !clickPlane) return;
  const p = boardPoint(e);
  if (p) sendObject(p.x, p.y);
});

window.addEventListener("resize", () => {
  camera.aspect = window.innerWidth / window.innerHeight;
  camera.updateProjectionMatrix();
  renderer.setSize(window.innerWidth, window.innerHeight);
});