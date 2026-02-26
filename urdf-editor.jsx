import { useState, useEffect, useRef, useCallback, useMemo } from "react";
import * as THREE from "three";

// ─── URDF Parser (supports multiple visuals per link) ──────────
function parseVisual(visual) {
  let origin = { xyz: [0, 0, 0], rpy: [0, 0, 0] };
  let geometry = null;
  let material = { color: [0.7, 0.7, 0.7, 1] };

  const originEl = visual.querySelector("origin");
  if (originEl) {
    const xyz = originEl.getAttribute("xyz");
    const rpy = originEl.getAttribute("rpy");
    if (xyz) origin.xyz = xyz.split(/\s+/).filter(Boolean).map(Number);
    if (rpy) origin.rpy = rpy.split(/\s+/).filter(Boolean).map(Number);
  }

  const geomEl = visual.querySelector("geometry");
  if (geomEl) {
    const box = geomEl.querySelector("box");
    const cyl = geomEl.querySelector("cylinder");
    const sph = geomEl.querySelector("sphere");
    if (box) {
      const size = box.getAttribute("size")?.split(/\s+/).filter(Boolean).map(Number) || [0.1, 0.1, 0.1];
      geometry = { type: "box", size };
    } else if (cyl) {
      geometry = {
        type: "cylinder",
        radius: parseFloat(cyl.getAttribute("radius") || 0.05),
        length: parseFloat(cyl.getAttribute("length") || 0.1),
      };
    } else if (sph) {
      geometry = { type: "sphere", radius: parseFloat(sph.getAttribute("radius") || 0.05) };
    }
  }

  const matEl = visual.querySelector("material");
  if (matEl) {
    const colorEl = matEl.querySelector("color");
    if (colorEl) {
      const rgba = colorEl.getAttribute("rgba")?.split(/\s+/).filter(Boolean).map(Number);
      if (rgba) material.color = rgba;
    }
  }

  return { origin, geometry, material };
}

function parseURDF(xml) {
  try {
    const parser = new DOMParser();
    const doc = parser.parseFromString(xml, "text/xml");
    const errorNode = doc.querySelector("parsererror");
    if (errorNode) return { error: errorNode.textContent, links: [], joints: [], name: "" };

    const robot = doc.querySelector("robot");
    if (!robot) return { error: "No <robot> element found", links: [], joints: [], name: "" };

    const name = robot.getAttribute("name") || "unnamed";
    const links = [];
    const joints = [];

    const directChildren = Array.from(robot.children);

    directChildren.filter((el) => el.tagName === "link").forEach((l) => {
      const linkName = l.getAttribute("name") || "unnamed_link";
      const visualEls = l.querySelectorAll("visual");
      const visuals = [];
      visualEls.forEach((v) => {
        const parsed = parseVisual(v);
        if (parsed.geometry) visuals.push(parsed);
      });
      links.push({ name: linkName, visuals });
    });

    directChildren.filter((el) => el.tagName === "joint").forEach((j) => {
      const jointName = j.getAttribute("name") || "unnamed_joint";
      const type = j.getAttribute("type") || "fixed";
      const parent = j.querySelector("parent")?.getAttribute("link") || "";
      const child = j.querySelector("child")?.getAttribute("link") || "";
      let origin = { xyz: [0, 0, 0], rpy: [0, 0, 0] };
      let axis = [0, 0, 1];
      let limit = null;

      const originEl = j.querySelector("origin");
      if (originEl) {
        const xyz = originEl.getAttribute("xyz");
        const rpy = originEl.getAttribute("rpy");
        if (xyz) origin.xyz = xyz.split(/\s+/).filter(Boolean).map(Number);
        if (rpy) origin.rpy = rpy.split(/\s+/).filter(Boolean).map(Number);
      }
      const axisEl = j.querySelector("axis");
      if (axisEl) {
        const xyz = axisEl.getAttribute("xyz");
        if (xyz) axis = xyz.split(/\s+/).filter(Boolean).map(Number);
      }
      const limitEl = j.querySelector("limit");
      if (limitEl) {
        limit = {
          lower: parseFloat(limitEl.getAttribute("lower") || 0),
          upper: parseFloat(limitEl.getAttribute("upper") || 0),
          effort: parseFloat(limitEl.getAttribute("effort") || 0),
          velocity: parseFloat(limitEl.getAttribute("velocity") || 0),
        };
      }
      joints.push({ name: jointName, type, parent, child, origin, axis, limit });
    });

    return { name, links, joints, error: null };
  } catch (e) {
    return { error: e.message, links: [], joints: [], name: "" };
  }
}

function buildTree(links, joints) {
  const linkMap = {};
  links.forEach((l) => (linkMap[l.name] = { ...l, children: [] }));
  const childSet = new Set();
  joints.forEach((j) => {
    childSet.add(j.child);
    if (linkMap[j.parent] && linkMap[j.child]) {
      linkMap[j.parent].children.push({ joint: j, link: linkMap[j.child] });
    }
  });
  const roots = links.filter((l) => !childSet.has(l.name)).map((l) => linkMap[l.name]);
  return roots.length > 0 ? roots : links.length > 0 ? [linkMap[links[0].name]] : [];
}

const DEFAULT_URDF = `<?xml version="1.0"?>
<robot name="z200">

  <link name="base_footprint" />

  <joint name="base_joint" type="fixed">
    <parent link="base_footprint" />
    <child link="base_link" />
    <origin xyz="0 0 0.85" rpy="0 0 0" />
  </joint>

  <link name="base_link">
    <visual>
      <geometry>
        <box size="1.52 1.05 1.10" />
      </geometry>
      <material name="light_gray">
        <color rgba="0.8 0.8 0.8 1.0" />
      </material>
    </visual>
    <visual>
      <geometry>
        <box size="1.52 1.05 0.05" />
      </geometry>
      <origin xyz="0 0 0.525" />
      <material name="dark_gray">
        <color rgba="0.4 0.4 0.4 1.0" />
      </material>
    </visual>
  </link>

  <link name="left_wheel_link">
    <visual>
      <origin rpy="1.5708 0 0" />
      <geometry>
        <cylinder radius="0.22" length="0.12" />
      </geometry>
      <material name="black">
        <color rgba="0.05 0.05 0.05 1.0" />
      </material>
    </visual>
  </link>

  <joint name="left_wheel_joint" type="continuous">
    <parent link="base_link" />
    <child link="left_wheel_link" />
    <origin xyz="-0.65 0.524 -0.487" />
    <axis xyz="0 1 0" />
  </joint>

  <link name="right_wheel_link">
    <visual>
      <origin rpy="1.5708 0 0" />
      <geometry>
        <cylinder radius="0.22" length="0.12" />
      </geometry>
      <material name="black">
        <color rgba="0.05 0.05 0.05 1.0" />
      </material>
    </visual>
  </link>

  <joint name="right_wheel_joint" type="continuous">
    <parent link="base_link" />
    <child link="right_wheel_link" />
    <origin xyz="-0.65 -0.524 -0.487" />
    <axis xyz="0 1 0" />
  </joint>

  <link name="front_steering_column">
    <visual>
      <geometry>
        <cylinder radius="0.05" length="0.3" />
      </geometry>
      <material name="black">
        <color rgba="0.05 0.05 0.05 1.0" />
      </material>
    </visual>
  </link>

  <joint name="front_steering_joint" type="revolute">
    <parent link="base_link" />
    <child link="front_steering_column" />
    <origin xyz="0.75 0 -0.487" />
    <axis xyz="0 0 1" />
    <limit lower="-0.8727" upper="0.8727" effort="100.0" velocity="2.0" />
  </joint>

  <link name="front_support_wheel_link">
    <visual>
      <origin rpy="1.5708 0 0" />
      <geometry>
        <cylinder radius="0.22" length="0.12" />
      </geometry>
      <material name="black">
        <color rgba="0.05 0.05 0.05 1.0" />
      </material>
    </visual>
  </link>

  <joint name="front_wheel_joint" type="continuous">
    <parent link="front_steering_column" />
    <child link="front_support_wheel_link" />
    <origin xyz="0 0 0" />
    <axis xyz="0 1 0" />
  </joint>

  <link name="corner_fl">
    <visual>
      <origin xyz="0.735 0.544 0.577" rpy="0 0 0.5236" />
      <geometry>
        <box size="0.28 0.16 0.06" />
      </geometry>
      <material name="dark_gray">
        <color rgba="0.4 0.4 0.4 1.0" />
      </material>
    </visual>
  </link>
  <joint name="corner_fl_joint" type="fixed">
    <parent link="base_link" />
    <child link="corner_fl" />
  </joint>

  <link name="corner_fr">
    <visual>
      <origin xyz="0.735 -0.544 0.577" rpy="0 0 -0.5236" />
      <geometry>
        <box size="0.28 0.16 0.06" />
      </geometry>
      <material name="dark_gray">
        <color rgba="0.4 0.4 0.4 1.0" />
      </material>
    </visual>
  </link>
  <joint name="corner_fr_joint" type="fixed">
    <parent link="base_link" />
    <child link="corner_fr" />
  </joint>

  <link name="corner_rl">
    <visual>
      <origin xyz="-0.735 0.544 0.577" rpy="0 0 -0.5236" />
      <geometry>
        <box size="0.28 0.16 0.06" />
      </geometry>
      <material name="dark_gray">
        <color rgba="0.4 0.4 0.4 1.0" />
      </material>
    </visual>
  </link>
  <joint name="corner_rl_joint" type="fixed">
    <parent link="base_link" />
    <child link="corner_rl" />
  </joint>

  <link name="corner_rr">
    <visual>
      <origin xyz="-0.735 -0.544 0.577" rpy="0 0 0.5236" />
      <geometry>
        <box size="0.28 0.16 0.06" />
      </geometry>
      <material name="dark_gray">
        <color rgba="0.4 0.4 0.4 1.0" />
      </material>
    </visual>
  </link>
  <joint name="corner_rr_joint" type="fixed">
    <parent link="base_link" />
    <child link="corner_rr" />
  </joint>

  <link name="laser_link">
    <visual>
      <geometry>
        <cylinder radius="0.05" length="0.1" />
      </geometry>
      <material name="red">
        <color rgba="1.0 0.0 0.0 1.0" />
      </material>
    </visual>
  </link>

  <joint name="laser_joint" type="fixed">
    <parent link="base_link" />
    <child link="laser_link" />
    <origin xyz="0.80 -0.605 0.125" rpy="0 0 0" />
  </joint>

  <link name="imu_link">
    <visual>
      <origin xyz="0 0 0.0" rpy="0 0 0" />
      <geometry>
        <box size="0.02 0.02 0.02" />
      </geometry>
    </visual>
  </link>

  <joint name="imu_joint" type="fixed">
    <parent link="base_link" />
    <child link="imu_link" />
    <origin xyz="0 0 0.55" rpy="0 0 0" />
  </joint>

</robot>`;

const JOINT_COLORS = {
  fixed: "#6b7280",
  revolute: "#f59e0b",
  continuous: "#10b981",
  prismatic: "#8b5cf6",
  floating: "#ec4899",
  planar: "#06b6d4",
};

const JOINT_ICONS = {
  fixed: "⊠",
  revolute: "↻",
  continuous: "⟳",
  prismatic: "↕",
  floating: "✦",
  planar: "◇",
};

function TreeNode({ node, depth = 0, selectedLink, onSelectLink }) {
  const [expanded, setExpanded] = useState(true);
  const hasChildren = node.children && node.children.length > 0;
  const visualCount = node.visuals?.length || 0;

  return (
    <div>
      <div
        onClick={() => onSelectLink(node.name)}
        style={{
          display: "flex", alignItems: "center", gap: 6, padding: "5px 8px",
          paddingLeft: 12 + depth * 16, cursor: "pointer",
          background: selectedLink === node.name ? "rgba(56,189,248,0.12)" : "transparent",
          borderLeft: selectedLink === node.name ? "2px solid #38bdf8" : "2px solid transparent",
          fontSize: 12, fontFamily: "'JetBrains Mono', 'Fira Code', monospace",
          color: selectedLink === node.name ? "#e2e8f0" : "#94a3b8", transition: "all 0.15s",
        }}
        onMouseEnter={(e) => { if (selectedLink !== node.name) e.currentTarget.style.background = "rgba(255,255,255,0.03)"; }}
        onMouseLeave={(e) => { if (selectedLink !== node.name) e.currentTarget.style.background = "transparent"; }}
      >
        {hasChildren ? (
          <span onClick={(e) => { e.stopPropagation(); setExpanded(!expanded); }}
            style={{ fontSize: 8, color: "#64748b", width: 10, textAlign: "center", userSelect: "none", cursor: "pointer" }}>
            {expanded ? "▼" : "▶"}
          </span>
        ) : <span style={{ width: 10 }} />}
        <span style={{ color: visualCount > 0 ? "#38bdf8" : "#334155", fontSize: 10 }}>■</span>
        <span style={{ flex: 1 }}>{node.name}</span>
        {visualCount > 1 && (
          <span style={{ fontSize: 9, color: "#475569", background: "#1e293b", borderRadius: 3, padding: "0 4px" }}>×{visualCount}</span>
        )}
        {visualCount === 1 && (
          <span style={{ fontSize: 9, color: "#475569", textTransform: "uppercase" }}>{node.visuals[0].geometry?.type}</span>
        )}
      </div>
      {expanded && hasChildren && node.children.map((ch, i) => (
        <div key={i}>
          <div style={{ display: "flex", alignItems: "center", gap: 6, padding: "3px 8px", paddingLeft: 22 + depth * 16, fontSize: 11, fontFamily: "'JetBrains Mono', 'Fira Code', monospace", color: "#64748b" }}>
            <span style={{ color: JOINT_COLORS[ch.joint.type] || "#6b7280", fontSize: 12 }}>{JOINT_ICONS[ch.joint.type] || "○"}</span>
            <span style={{ color: "#78716c" }}>{ch.joint.name}</span>
            <span style={{ fontSize: 9, padding: "1px 5px", borderRadius: 3, background: (JOINT_COLORS[ch.joint.type] || "#6b7280") + "22", color: JOINT_COLORS[ch.joint.type] }}>{ch.joint.type}</span>
          </div>
          <TreeNode node={ch.link} depth={depth + 1} selectedLink={selectedLink} onSelectLink={onSelectLink} />
        </div>
      ))}
    </div>
  );
}


function Viewport({ parsedData, selectedLink, onSelectLink, jointValues }) {
  const containerRef = useRef(null);
  const sceneRef = useRef(null);
  const rendererRef = useRef(null);
  const cameraRef = useRef(null);
  const meshMapRef = useRef({});
  const robotGroupRef = useRef(null);
  const frameRef = useRef(null);
  const mouseRef = useRef({ isDown: false, button: -1, x: 0, y: 0 });
  const orbitRef = useRef({ theta: Math.PI / 4, phi: Math.PI / 3, distance: 4, target: new THREE.Vector3(0, 0.8, 0) });
  const jointDataRef = useRef([]);
  const markerGroupRef = useRef(null);

  const setupScene = useCallback(() => {
    if (!containerRef.current) return;
    const w = containerRef.current.clientWidth;
    const h = containerRef.current.clientHeight;

    const scene = new THREE.Scene();
    scene.background = new THREE.Color(0x0f1219);
    scene.fog = new THREE.Fog(0x0f1219, 15, 40);

    const camera = new THREE.PerspectiveCamera(50, w / h, 0.01, 100);
    const renderer = new THREE.WebGLRenderer({ antialias: true });
    renderer.setSize(w, h);
    renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
    renderer.shadowMap.enabled = true;
    renderer.shadowMap.type = THREE.PCFSoftShadowMap;
    renderer.toneMapping = THREE.ACESFilmicToneMapping;
    renderer.toneMappingExposure = 0.95;
    containerRef.current.appendChild(renderer.domElement);

    scene.add(new THREE.GridHelper(20, 40, 0x1e293b, 0x151c28));

    const ground = new THREE.Mesh(
      new THREE.PlaneGeometry(40, 40),
      new THREE.MeshStandardMaterial({ color: 0x0f1219, roughness: 0.9, metalness: 0.1 })
    );
    ground.rotation.x = -Math.PI / 2;
    ground.position.y = -0.002;
    ground.receiveShadow = true;
    scene.add(ground);

    scene.add(new THREE.AmbientLight(0x667788, 1.0));

    const dirLight = new THREE.DirectionalLight(0xffeedd, 1.2);
    dirLight.position.set(5, 8, 4);
    dirLight.castShadow = true;
    dirLight.shadow.mapSize.width = 2048;
    dirLight.shadow.mapSize.height = 2048;
    dirLight.shadow.camera.near = 0.5;
    dirLight.shadow.camera.far = 30;
    dirLight.shadow.camera.left = -6;
    dirLight.shadow.camera.right = 6;
    dirLight.shadow.camera.top = 6;
    dirLight.shadow.camera.bottom = -6;
    scene.add(dirLight);

    const fill = new THREE.DirectionalLight(0x4488cc, 0.8);
    fill.position.set(-4, 5, -2);
    scene.add(fill);
    const rim = new THREE.DirectionalLight(0x88aaff, 0.5);
    rim.position.set(0, 2, -6);
    scene.add(rim);

    sceneRef.current = scene;
    rendererRef.current = renderer;
    cameraRef.current = camera;
  }, []);

  // Build robot geometry — ONLY on parsedData change (not jointValues)
  const buildRobotMeshes = useCallback(() => {
    const scene = sceneRef.current;
    if (!scene || !parsedData) return;

    if (robotGroupRef.current) {
      scene.remove(robotGroupRef.current);
      robotGroupRef.current.traverse((c) => {
        if (c.geometry) c.geometry.dispose();
        if (c.material) {
          if (Array.isArray(c.material)) c.material.forEach((m) => m.dispose());
          else c.material.dispose();
        }
      });
    }
    meshMapRef.current = {};
    jointDataRef.current = [];

    const { links, joints } = parsedData;
    if (!links.length) return;

    const linkGroups = {};
    links.forEach((l) => {
      const group = new THREE.Group();
      group.name = l.name;
      linkGroups[l.name] = group;

      (l.visuals || []).forEach((vis) => {
        if (!vis.geometry) return;
        let geom;
        const g = vis.geometry;
        if (g.type === "box") geom = new THREE.BoxGeometry(g.size[0], g.size[1], g.size[2]);
        else if (g.type === "cylinder") {
          geom = new THREE.CylinderGeometry(g.radius, g.radius, g.length, 32);
          geom.rotateX(Math.PI / 2);
        }
        else if (g.type === "sphere") geom = new THREE.SphereGeometry(g.radius, 32, 20);

        if (geom) {
          const col = vis.material?.color || [0.7, 0.7, 0.7, 1];
          const mat = new THREE.MeshStandardMaterial({
            color: new THREE.Color(col[0], col[1], col[2]),
            roughness: 0.7, metalness: 0.05,
            transparent: col[3] < 1, opacity: col[3],
          });
          const mesh = new THREE.Mesh(geom, mat);
          mesh.castShadow = true;
          mesh.receiveShadow = true;
          mesh.userData.linkName = l.name;

          if (vis.origin) {
            mesh.position.set(vis.origin.xyz[0], vis.origin.xyz[1], vis.origin.xyz[2]);
            mesh.rotation.set(vis.origin.rpy[0], vis.origin.rpy[1], vis.origin.rpy[2], "XYZ");
          }
          group.add(mesh);

          const wire = new THREE.LineSegments(
            new THREE.EdgesGeometry(geom),
            new THREE.LineBasicMaterial({ color: 0x38bdf8, transparent: true, opacity: 0.02 })
          );
          wire.position.copy(mesh.position);
          wire.rotation.copy(mesh.rotation);
          group.add(wire);
        }
      });

      const axLen = 0.05;
      [{ dir: [axLen, 0, 0], color: 0xff4444 }, { dir: [0, axLen, 0], color: 0x44ff44 }, { dir: [0, 0, axLen], color: 0x4444ff }].forEach(({ dir, color }) => {
        const pts = [new THREE.Vector3(0, 0, 0), new THREE.Vector3(...dir)];
        group.add(new THREE.Line(new THREE.BufferGeometry().setFromPoints(pts), new THREE.LineBasicMaterial({ color, transparent: true, opacity: 0.25 })));
      });
    });

    // Assemble kinematic chain with default (zero) transforms
    const childSet = new Set();
    joints.forEach((j) => {
      childSet.add(j.child);
      const parentGroup = linkGroups[j.parent];
      const childGroup = linkGroups[j.child];
      if (parentGroup && childGroup) {
        // Set base transform (will be updated by updateJointTransforms)
        childGroup.position.set(j.origin.xyz[0], j.origin.xyz[1], j.origin.xyz[2]);
        childGroup.quaternion.setFromEuler(new THREE.Euler(j.origin.rpy[0], j.origin.rpy[1], j.origin.rpy[2], "XYZ"));
        parentGroup.add(childGroup);

        // Store joint metadata for cheap transform updates
        jointDataRef.current.push({
          name: j.name, type: j.type, axis: j.axis, origin: j.origin,
          childGroup, parentGroup,
        });

        // Static connector line
        const jColor = JOINT_COLORS[j.type] ? parseInt(JOINT_COLORS[j.type].replace("#", ""), 16) : 0x64748b;
        const len = new THREE.Vector3(j.origin.xyz[0], j.origin.xyz[1], j.origin.xyz[2]).length();
        if (len > 0.001) {
          const connGeo = new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(0, 0, 0), new THREE.Vector3(j.origin.xyz[0], j.origin.xyz[1], j.origin.xyz[2])]);
          const connLine = new THREE.Line(connGeo, new THREE.LineDashedMaterial({ color: jColor, transparent: true, opacity: 0.3, dashSize: 0.03, gapSize: 0.015 }));
          connLine.computeLineDistances();
          parentGroup.add(connLine);
        }
      }
    });

    const robotGroup = new THREE.Group();
    robotGroup.rotation.x = -Math.PI / 2; // URDF Z-up to Three.js Y-up

    // Marker group — separate child of robotGroup, rebuilt cheaply on joint changes
    const mGroup = new THREE.Group();
    mGroup.name = "joint_markers";
    robotGroup.add(mGroup);
    markerGroupRef.current = mGroup;

    links.forEach((l) => { if (!childSet.has(l.name)) robotGroup.add(linkGroups[l.name]); });
    scene.add(robotGroup);
    robotGroupRef.current = robotGroup;
    meshMapRef.current = linkGroups;

    // Auto-fit camera to model + ground the robot
    robotGroup.updateMatrixWorld(true);
    const bbox = new THREE.Box3().setFromObject(robotGroup);
    if (!bbox.isEmpty()) {
      // Shift robot down so its lowest point sits on the ground (y=0)
      robotGroup.position.y = -bbox.min.y;
      robotGroup.updateMatrixWorld(true);
      bbox.setFromObject(robotGroup);

      const center = new THREE.Vector3();
      const size = new THREE.Vector3();
      bbox.getCenter(center);
      bbox.getSize(size);
      orbitRef.current.target.copy(center);
      orbitRef.current.distance = Math.max(Math.max(size.x, size.y, size.z) * 2, 1.5);
    }
  }, [parsedData]); // ← NO jointValues here

  // Update joint transforms + markers — cheap, runs on every slider change
  const updateJointTransforms = useCallback(() => {
    if (!jointDataRef.current.length) return;

    // Update kinematic chain transforms
    jointDataRef.current.forEach((jd) => {
      const { childGroup, origin, type, axis, name } = jd;
      const jVal = jointValues?.[name] ?? 0;

      childGroup.position.set(origin.xyz[0], origin.xyz[1], origin.xyz[2]);
      const baseQuat = new THREE.Quaternion().setFromEuler(new THREE.Euler(origin.rpy[0], origin.rpy[1], origin.rpy[2], "XYZ"));

      if (jVal !== 0 && (type === "revolute" || type === "continuous")) {
        baseQuat.multiply(new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(...axis).normalize(), jVal));
      }
      if (jVal !== 0 && type === "prismatic") {
        childGroup.position.add(new THREE.Vector3(...axis).normalize().multiplyScalar(jVal));
      }
      childGroup.quaternion.copy(baseQuat);
    });

    // Rebuild markers (lightweight — just torus, lines, cones)
    const mGroup = markerGroupRef.current;
    if (!mGroup) return;

    while (mGroup.children.length > 0) {
      const ch = mGroup.children[0];
      ch.traverse((c) => { if (c.geometry) c.geometry.dispose(); if (c.material) c.material.dispose(); });
      mGroup.remove(ch);
    }

    jointDataRef.current.forEach((jd) => {
      const { childGroup, type, axis: jAxis, name } = jd;
      if (type !== "revolute" && type !== "continuous" && type !== "prismatic") return;

      const jColor = JOINT_COLORS[type] ? parseInt(JOINT_COLORS[type].replace("#", ""), 16) : 0x64748b;
      const jVal = jointValues?.[name] ?? 0;

      // Get marker position in robotGroup-local space
      const mg = new THREE.Group();
      childGroup.updateMatrixWorld(true);
      robotGroupRef.current.updateMatrixWorld(true);
      const wPos = new THREE.Vector3();
      childGroup.getWorldPosition(wPos);
      mg.position.copy(robotGroupRef.current.worldToLocal(wPos.clone()));
      // Inherit parent orientation (markers don't rotate with the joint itself)
      const parentWQ = new THREE.Quaternion();
      if (childGroup.parent) childGroup.parent.getWorldQuaternion(parentWQ);
      const robotWQ = new THREE.Quaternion();
      robotGroupRef.current.getWorldQuaternion(robotWQ);
      mg.quaternion.copy(robotWQ.clone().invert().multiply(parentWQ));

      if (type === "revolute" || type === "continuous") {
        const ringRadius = 0.3;
        const axVec = new THREE.Vector3(...jAxis).normalize();
        const colorObj = new THREE.Color(jColor);
        const ringQuat = new THREE.Quaternion().setFromUnitVectors(new THREE.Vector3(0, 0, 1), axVec);

        // Torus ring
        const torus = new THREE.Mesh(
          new THREE.TorusGeometry(ringRadius, 0.008, 8, 64),
          new THREE.MeshBasicMaterial({ color: colorObj, transparent: true, opacity: 0.85 })
        );
        torus.quaternion.copy(ringQuat);
        mg.add(torus);

        // Swept arc + wedge showing current value
        if (Math.abs(jVal) > 0.01) {
          const arcSegs = Math.max(8, Math.round(Math.abs(jVal) / (Math.PI * 2) * 64));
          const arcPts = [];
          for (let s = 0; s <= arcSegs; s++) {
            const a = (s / arcSegs) * jVal;
            arcPts.push(new THREE.Vector3(Math.cos(a) * ringRadius, Math.sin(a) * ringRadius, 0));
          }
          const arc = new THREE.Line(new THREE.BufferGeometry().setFromPoints(arcPts), new THREE.LineBasicMaterial({ color: 0xffffff }));
          arc.quaternion.copy(ringQuat);
          mg.add(arc);

          const shape = new THREE.Shape();
          shape.moveTo(0, 0);
          for (let s = 0; s <= arcSegs; s++) {
            const a = (s / arcSegs) * jVal;
            shape.lineTo(Math.cos(a) * ringRadius * 0.9, Math.sin(a) * ringRadius * 0.9);
          }
          shape.lineTo(0, 0);
          const wedge = new THREE.Mesh(new THREE.ShapeGeometry(shape), new THREE.MeshBasicMaterial({ color: colorObj, transparent: true, opacity: 0.2, side: THREE.DoubleSide }));
          wedge.quaternion.copy(ringQuat);
          mg.add(wedge);
        }

        // Arrow cone at current angle
        const arrowAngle = jVal || 0;
        const coneGeo = new THREE.ConeGeometry(0.02, 0.06, 8);
        coneGeo.translate(0, 0.03, 0);
        coneGeo.rotateZ(-Math.PI / 2);
        const cone = new THREE.Mesh(coneGeo, new THREE.MeshBasicMaterial({ color: 0xffffff }));
        cone.position.set(Math.cos(arrowAngle) * ringRadius, Math.sin(arrowAngle) * ringRadius, 0);
        cone.quaternion.setFromUnitVectors(new THREE.Vector3(1, 0, 0), new THREE.Vector3(-Math.sin(arrowAngle), Math.cos(arrowAngle), 0));
        const coneW = new THREE.Group();
        coneW.quaternion.copy(ringQuat);
        coneW.add(cone);
        mg.add(coneW);

        // Zero tick
        const refLine = new THREE.Line(
          new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(ringRadius * 0.7, 0, 0), new THREE.Vector3(ringRadius * 1.1, 0, 0)]),
          new THREE.LineBasicMaterial({ color: 0xffffff, transparent: true, opacity: 0.4 })
        );
        refLine.quaternion.copy(ringQuat);
        mg.add(refLine);

        // Center dot + axis line
        mg.add(new THREE.Mesh(new THREE.SphereGeometry(0.025, 12, 8), new THREE.MeshBasicMaterial({ color: colorObj })));
        mg.add(new THREE.Line(
          new THREE.BufferGeometry().setFromPoints([axVec.clone().multiplyScalar(-0.15), axVec.clone().multiplyScalar(0.15)]),
          new THREE.LineBasicMaterial({ color: jColor, transparent: true, opacity: 0.5 })
        ));
      }

      if (type === "prismatic") {
        const axVec = new THREE.Vector3(...jAxis).normalize();
        mg.add(new THREE.Line(
          new THREE.BufferGeometry().setFromPoints([axVec.clone().multiplyScalar(-0.25), axVec.clone().multiplyScalar(0.25)]),
          new THREE.LineBasicMaterial({ color: jColor, transparent: true, opacity: 0.7 })
        ));
        const cGeo = new THREE.ConeGeometry(0.02, 0.06, 8);
        cGeo.translate(0, 0.03, 0);
        const cMesh = new THREE.Mesh(cGeo, new THREE.MeshBasicMaterial({ color: jColor }));
        cMesh.position.copy(axVec.clone().multiplyScalar(0.25));
        cMesh.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), axVec);
        mg.add(cMesh);
        mg.add(new THREE.Mesh(new THREE.SphereGeometry(0.025, 12, 8), new THREE.MeshBasicMaterial({ color: jColor })));
      }

      mGroup.add(mg);
    });
  }, [jointValues]);

  // Highlight selected link
  useEffect(() => {
    Object.entries(meshMapRef.current).forEach(([name, group]) => {
      group.traverse((c) => {
        if (c.isMesh && c.material && c.material.emissive) {
          c.material.emissive = name === selectedLink ? new THREE.Color(0x38bdf8) : new THREE.Color(0x000000);
          c.material.emissiveIntensity = name === selectedLink ? 0.2 : 0;
        }
      });
    });
  }, [selectedLink]);

  // Setup scene once
  useEffect(() => {
    setupScene();
    return () => {
      if (frameRef.current) cancelAnimationFrame(frameRef.current);
      if (rendererRef.current && containerRef.current) {
        try { containerRef.current.removeChild(rendererRef.current.domElement); } catch (_) {}
        rendererRef.current.dispose();
      }
    };
  }, [setupScene]);

  // Build meshes when URDF changes
  useEffect(() => { buildRobotMeshes(); }, [buildRobotMeshes]);

  // Update transforms when joint values change (cheap — no geometry rebuild)
  useEffect(() => { updateJointTransforms(); }, [updateJointTransforms]);

  // Render loop
  useEffect(() => {
    const animate = () => {
      frameRef.current = requestAnimationFrame(animate);
      if (!cameraRef.current || !rendererRef.current || !sceneRef.current) return;
      const { theta, phi, distance, target } = orbitRef.current;
      cameraRef.current.position.set(
        target.x + distance * Math.sin(phi) * Math.cos(theta),
        target.y + distance * Math.cos(phi),
        target.z + distance * Math.sin(phi) * Math.sin(theta)
      );
      cameraRef.current.lookAt(target);
      rendererRef.current.render(sceneRef.current, cameraRef.current);
    };
    animate();
    return () => { if (frameRef.current) cancelAnimationFrame(frameRef.current); };
  }, []);

  // Orbit controls
  useEffect(() => {
    const el = containerRef.current;
    if (!el) return;
    const onMouseDown = (e) => { mouseRef.current = { isDown: true, button: e.button, x: e.clientX, y: e.clientY }; };
    const onMouseMove = (e) => {
      if (!mouseRef.current.isDown) return;
      const dx = e.clientX - mouseRef.current.x;
      const dy = e.clientY - mouseRef.current.y;
      mouseRef.current.x = e.clientX;
      mouseRef.current.y = e.clientY;
      if (mouseRef.current.button === 0) {
        orbitRef.current.theta -= dx * 0.005;
        orbitRef.current.phi = Math.max(0.1, Math.min(Math.PI - 0.1, orbitRef.current.phi - dy * 0.005));
      } else if (mouseRef.current.button === 2) {
        const cam = cameraRef.current;
        if (!cam) return;
        const fwd = new THREE.Vector3().subVectors(cam.position, orbitRef.current.target).normalize();
        const right = new THREE.Vector3().crossVectors(cam.up, fwd).normalize();
        const up = new THREE.Vector3().crossVectors(fwd, right).normalize();
        const scale = orbitRef.current.distance * 0.001;
        orbitRef.current.target.add(right.multiplyScalar(-dx * scale));
        orbitRef.current.target.add(up.multiplyScalar(dy * scale));
      }
    };
    const onMouseUp = () => { mouseRef.current.isDown = false; };
    const onWheel = (e) => {
      e.preventDefault();
      if (mouseRef.current.isDown) return;
      if (Math.abs(e.deltaY) < 2) return;
      orbitRef.current.distance = Math.max(0.3, Math.min(30, orbitRef.current.distance * (1 + e.deltaY * 0.001)));
    };
    const onContext = (e) => e.preventDefault();
    el.addEventListener("mousedown", onMouseDown);
    window.addEventListener("mousemove", onMouseMove);
    window.addEventListener("mouseup", onMouseUp);
    el.addEventListener("wheel", onWheel, { passive: false });
    el.addEventListener("contextmenu", onContext);
    return () => {
      el.removeEventListener("mousedown", onMouseDown);
      window.removeEventListener("mousemove", onMouseMove);
      window.removeEventListener("mouseup", onMouseUp);
      el.removeEventListener("wheel", onWheel);
      el.removeEventListener("contextmenu", onContext);
    };
  }, []);

  // Resize observer
  useEffect(() => {
    const ro = new ResizeObserver(() => {
      if (!containerRef.current || !rendererRef.current || !cameraRef.current) return;
      const w = containerRef.current.clientWidth;
      const h = containerRef.current.clientHeight;
      rendererRef.current.setSize(w, h);
      cameraRef.current.aspect = w / h;
      cameraRef.current.updateProjectionMatrix();
    });
    if (containerRef.current) ro.observe(containerRef.current);
    return () => ro.disconnect();
  }, []);

  return <div ref={containerRef} style={{ width: "100%", height: "100%", cursor: "grab" }} />;
}
function CodeEditor({ code, onChange }) {
  const textareaRef = useRef(null);
  const lineCountRef = useRef(null);
  const lineCount = useMemo(() => code.split("\n").length, [code]);
  const syncScroll = () => { if (textareaRef.current && lineCountRef.current) lineCountRef.current.scrollTop = textareaRef.current.scrollTop; };

  return (
    <div style={{ display: "flex", height: "100%", background: "#0b0e14", fontFamily: "'JetBrains Mono', 'Fira Code', monospace" }}>
      <div ref={lineCountRef} style={{ width: 44, padding: "12px 8px 12px 0", textAlign: "right", fontSize: 11, lineHeight: "18px", color: "#334155", userSelect: "none", overflow: "hidden", borderRight: "1px solid #1e293b", flexShrink: 0 }}>
        {Array.from({ length: lineCount }, (_, i) => <div key={i}>{i + 1}</div>)}
      </div>
      <textarea ref={textareaRef} value={code} onChange={(e) => onChange(e.target.value)} onScroll={syncScroll} spellCheck={false}
        style={{ flex: 1, background: "transparent", color: "#cbd5e1", border: "none", outline: "none", resize: "none", padding: "12px", fontSize: 12, lineHeight: "18px", fontFamily: "inherit", tabSize: 2, whiteSpace: "pre", overflowWrap: "normal", overflowX: "auto" }} />
    </div>
  );
}

function AddLinkDialog({ onAdd, onClose }) {
  const [name, setName] = useState("");
  const [shape, setShape] = useState("box");
  const [sx, setSx] = useState("0.1"); const [sy, setSy] = useState("0.1"); const [sz, setSz] = useState("0.1");
  const [radius, setRadius] = useState("0.05"); const [length, setLength] = useState("0.1");
  const [color, setColor] = useState("#5588bb");

  const generate = () => {
    if (!name.trim()) return;
    const c = [parseInt(color.slice(1, 3), 16) / 255, parseInt(color.slice(3, 5), 16) / 255, parseInt(color.slice(5, 7), 16) / 255];
    let geom = "";
    if (shape === "box") geom = '<box size="' + sx + " " + sy + " " + sz + '"/>';
    else if (shape === "cylinder") geom = '<cylinder radius="' + radius + '" length="' + length + '"/>';
    else if (shape === "sphere") geom = '<sphere radius="' + radius + '"/>';
    const xml = '\n  <link name="' + name.trim() + '">\n    <visual>\n      <geometry>\n        ' + geom + '\n      </geometry>\n      <material name="mat_' + name.trim() + '">\n        <color rgba="' + c[0].toFixed(2) + " " + c[1].toFixed(2) + " " + c[2].toFixed(2) + ' 1"/>\n      </material>\n    </visual>\n  </link>';
    onAdd(xml);
  };

  const inputStyle = { background: "#0f1219", border: "1px solid #1e293b", borderRadius: 4, color: "#e2e8f0", padding: "6px 8px", fontSize: 12, fontFamily: "'JetBrains Mono', monospace", outline: "none", width: "100%", boxSizing: "border-box" };
  const labelStyle = { fontSize: 10, color: "#64748b", textTransform: "uppercase", letterSpacing: 1, marginBottom: 4 };

  return (
    <div style={{ position: "absolute", top: 0, left: 0, right: 0, bottom: 0, background: "rgba(0,0,0,0.6)", display: "flex", alignItems: "center", justifyContent: "center", zIndex: 100 }}>
      <div style={{ background: "#161b26", border: "1px solid #1e293b", borderRadius: 8, padding: 24, width: 340 }}>
        <div style={{ fontSize: 14, color: "#e2e8f0", marginBottom: 16, fontWeight: 600 }}>Add Link</div>
        <div style={{ marginBottom: 12 }}><div style={labelStyle}>Link Name</div><input value={name} onChange={(e) => setName(e.target.value)} placeholder="my_link" style={inputStyle} /></div>
        <div style={{ marginBottom: 12 }}>
          <div style={labelStyle}>Shape</div>
          <div style={{ display: "flex", gap: 4 }}>
            {["box", "cylinder", "sphere"].map((s) => (
              <button key={s} onClick={() => setShape(s)} style={{ flex: 1, padding: "6px 0", fontSize: 11, borderRadius: 4, cursor: "pointer", background: shape === s ? "#1e3a5f" : "#0f1219", border: shape === s ? "1px solid #38bdf8" : "1px solid #1e293b", color: shape === s ? "#38bdf8" : "#64748b", fontFamily: "'JetBrains Mono', monospace" }}>{s}</button>
            ))}
          </div>
        </div>
        {shape === "box" && (
          <div style={{ display: "flex", gap: 8, marginBottom: 12 }}>
            {[["X", sx, setSx], ["Y", sy, setSy], ["Z", sz, setSz]].map(([l, v, set]) => (
              <div key={l} style={{ flex: 1 }}><div style={labelStyle}>{l}</div><input value={v} onChange={(e) => set(e.target.value)} style={inputStyle} /></div>
            ))}
          </div>
        )}
        {(shape === "cylinder" || shape === "sphere") && (
          <div style={{ display: "flex", gap: 8, marginBottom: 12 }}>
            <div style={{ flex: 1 }}><div style={labelStyle}>Radius</div><input value={radius} onChange={(e) => setRadius(e.target.value)} style={inputStyle} /></div>
            {shape === "cylinder" && <div style={{ flex: 1 }}><div style={labelStyle}>Length</div><input value={length} onChange={(e) => setLength(e.target.value)} style={inputStyle} /></div>}
          </div>
        )}
        <div style={{ marginBottom: 16 }}><div style={labelStyle}>Color</div><input type="color" value={color} onChange={(e) => setColor(e.target.value)} style={{ width: 40, height: 28, border: "1px solid #1e293b", background: "transparent", cursor: "pointer", borderRadius: 4 }} /></div>
        <div style={{ display: "flex", gap: 8, justifyContent: "flex-end" }}>
          <button onClick={onClose} style={{ padding: "6px 16px", background: "#1e293b", border: "none", borderRadius: 4, color: "#94a3b8", cursor: "pointer", fontSize: 12 }}>Cancel</button>
          <button onClick={generate} style={{ padding: "6px 16px", background: "#1e3a5f", border: "1px solid #38bdf8", borderRadius: 4, color: "#38bdf8", cursor: "pointer", fontSize: 12 }}>Add</button>
        </div>
      </div>
    </div>
  );
}

function AddJointDialog({ links, onAdd, onClose }) {
  const [name, setName] = useState(""); const [type, setType] = useState("fixed");
  const [parent, setParent] = useState(links[0]?.name || ""); const [child, setChild] = useState(links[1]?.name || links[0]?.name || "");
  const [ox, setOx] = useState("0"); const [oy, setOy] = useState("0"); const [oz, setOz] = useState("0.1");
  const [ax, setAx] = useState("0"); const [ay, setAy] = useState("0"); const [az, setAz] = useState("1");

  const generate = () => {
    if (!name.trim() || !parent || !child) return;
    let extra = "";
    if (type !== "fixed") {
      extra = '\n    <axis xyz="' + ax + " " + ay + " " + az + '"/>';
      if (type === "revolute" || type === "prismatic") extra += '\n    <limit lower="-1.57" upper="1.57" effort="10" velocity="1.0"/>';
    }
    const xml = '\n  <joint name="' + name.trim() + '" type="' + type + '">\n    <parent link="' + parent + '"/>\n    <child link="' + child + '"/>\n    <origin xyz="' + ox + " " + oy + " " + oz + '" rpy="0 0 0"/>' + extra + '\n  </joint>';
    onAdd(xml);
  };

  const inputStyle = { background: "#0f1219", border: "1px solid #1e293b", borderRadius: 4, color: "#e2e8f0", padding: "6px 8px", fontSize: 12, fontFamily: "'JetBrains Mono', monospace", outline: "none", width: "100%", boxSizing: "border-box" };
  const selectStyle = { ...inputStyle, cursor: "pointer" };
  const labelStyle = { fontSize: 10, color: "#64748b", textTransform: "uppercase", letterSpacing: 1, marginBottom: 4 };

  return (
    <div style={{ position: "absolute", top: 0, left: 0, right: 0, bottom: 0, background: "rgba(0,0,0,0.6)", display: "flex", alignItems: "center", justifyContent: "center", zIndex: 100 }}>
      <div style={{ background: "#161b26", border: "1px solid #1e293b", borderRadius: 8, padding: 24, width: 340 }}>
        <div style={{ fontSize: 14, color: "#e2e8f0", marginBottom: 16, fontWeight: 600 }}>Add Joint</div>
        <div style={{ marginBottom: 12 }}><div style={labelStyle}>Joint Name</div><input value={name} onChange={(e) => setName(e.target.value)} placeholder="my_joint" style={inputStyle} /></div>
        <div style={{ marginBottom: 12 }}>
          <div style={labelStyle}>Type</div>
          <select value={type} onChange={(e) => setType(e.target.value)} style={selectStyle}>
            {["fixed", "revolute", "continuous", "prismatic", "floating", "planar"].map((t) => <option key={t} value={t}>{t}</option>)}
          </select>
        </div>
        <div style={{ display: "flex", gap: 8, marginBottom: 12 }}>
          <div style={{ flex: 1 }}><div style={labelStyle}>Parent</div><select value={parent} onChange={(e) => setParent(e.target.value)} style={selectStyle}>{links.map((l) => <option key={l.name} value={l.name}>{l.name}</option>)}</select></div>
          <div style={{ flex: 1 }}><div style={labelStyle}>Child</div><select value={child} onChange={(e) => setChild(e.target.value)} style={selectStyle}>{links.map((l) => <option key={l.name} value={l.name}>{l.name}</option>)}</select></div>
        </div>
        <div style={{ marginBottom: 12 }}><div style={labelStyle}>Origin XYZ</div><div style={{ display: "flex", gap: 8 }}>{[[ox, setOx], [oy, setOy], [oz, setOz]].map(([v, set], i) => <input key={i} value={v} onChange={(e) => set(e.target.value)} style={{ ...inputStyle, flex: 1 }} />)}</div></div>
        {type !== "fixed" && <div style={{ marginBottom: 12 }}><div style={labelStyle}>Axis XYZ</div><div style={{ display: "flex", gap: 8 }}>{[[ax, setAx], [ay, setAy], [az, setAz]].map(([v, set], i) => <input key={i} value={v} onChange={(e) => set(e.target.value)} style={{ ...inputStyle, flex: 1 }} />)}</div></div>}
        <div style={{ display: "flex", gap: 8, justifyContent: "flex-end" }}>
          <button onClick={onClose} style={{ padding: "6px 16px", background: "#1e293b", border: "none", borderRadius: 4, color: "#94a3b8", cursor: "pointer", fontSize: 12 }}>Cancel</button>
          <button onClick={generate} style={{ padding: "6px 16px", background: "#1e3a5f", border: "1px solid #38bdf8", borderRadius: 4, color: "#38bdf8", cursor: "pointer", fontSize: 12 }}>Add</button>
        </div>
      </div>
    </div>
  );
}

// ─── Connection Graph (SVG) ────────────────────────────────────
function ConnectionGraph({ parsedData, selectedLink, onSelectLink }) {
  const containerRef = useRef(null);
  const [pan, setPan] = useState({ x: 0, y: 0 });
  const [zoom, setZoom] = useState(1);
  const dragRef = useRef({ active: false, sx: 0, sy: 0, px: 0, py: 0 });

  const layout = useMemo(() => {
    if (!parsedData || parsedData.error) return { nodes: [], edges: [] };
    const { links, joints } = parsedData;
    if (!links.length) return { nodes: [], edges: [] };

    const childMap = {};
    joints.forEach((j) => {
      if (!childMap[j.parent]) childMap[j.parent] = [];
      childMap[j.parent].push({ joint: j, child: j.child });
    });

    const childSet = new Set(joints.map((j) => j.child));
    const roots = links.filter((l) => !childSet.has(l.name)).map((l) => l.name);
    if (roots.length === 0 && links.length > 0) roots.push(links[0].name);

    const nodePos = {};
    const depthCounts = {};
    const queue = roots.map((r) => ({ name: r, depth: 0 }));
    const visited = new Set();

    while (queue.length > 0) {
      const { name, depth } = queue.shift();
      if (visited.has(name)) continue;
      visited.add(name);
      if (!depthCounts[depth]) depthCounts[depth] = 0;
      nodePos[name] = { depth, index: depthCounts[depth]++ };
      (childMap[name] || []).forEach((ch) => {
        if (!visited.has(ch.child)) queue.push({ name: ch.child, depth: depth + 1 });
      });
    }
    links.forEach((l) => {
      if (!nodePos[l.name]) {
        if (!depthCounts[0]) depthCounts[0] = 0;
        nodePos[l.name] = { depth: 0, index: depthCounts[0]++ };
      }
    });

    const gapX = 150, gapY = 56;
    const maxDepth = Math.max(...Object.values(nodePos).map((n) => n.depth), 0);
    const nodes = [];
    for (let d = 0; d <= maxDepth; d++) {
      const atDepth = Object.entries(nodePos).filter(([, v]) => v.depth === d);
      const total = atDepth.length * gapX;
      atDepth.forEach(([name, pos]) => {
        const x = pos.index * gapX - total / 2 + gapX / 2;
        const y = pos.depth * gapY;
        const hasVis = links.find((l) => l.name === name)?.visuals?.length > 0;
        nodes.push({ name, x, y, hasVis });
      });
    }

    const nm = {};
    nodes.forEach((n) => (nm[n.name] = n));
    const edges = [];
    joints.forEach((j) => {
      const from = nm[j.parent];
      const to = nm[j.child];
      if (from && to) edges.push({ from, to, joint: j });
    });
    return { nodes, edges };
  }, [parsedData]);

  useEffect(() => {
    if (!containerRef.current || !layout.nodes.length) return;
    const rect = containerRef.current.getBoundingClientRect();
    const xs = layout.nodes.map((n) => n.x);
    const ys = layout.nodes.map((n) => n.y);
    const gW = (Math.max(...xs) - Math.min(...xs)) + 200;
    const gH = (Math.max(...ys) - Math.min(...ys)) + 80;
    const z = Math.min(rect.width / gW, rect.height / gH, 1.4) * 0.85;
    const cx = (Math.min(...xs) + Math.max(...xs)) / 2;
    const cy = (Math.min(...ys) + Math.max(...ys)) / 2;
    setZoom(z);
    setPan({ x: rect.width / 2 - cx * z, y: rect.height / 2 - cy * z });
  }, [layout]);

  const onDown = (e) => {
    if (e.target.closest(".graph-node")) return;
    dragRef.current = { active: true, sx: e.clientX, sy: e.clientY, px: pan.x, py: pan.y };
  };
  const onMoveG = useCallback((e) => {
    const d = dragRef.current;
    if (!d.active) return;
    setPan({ x: d.px + e.clientX - d.sx, y: d.py + e.clientY - d.sy });
  }, []);
  const onUpG = useCallback(() => { dragRef.current.active = false; }, []);
  const onWheelG = useCallback((e) => {
    e.preventDefault();
    setZoom((z) => Math.max(0.2, Math.min(3, z * (1 - e.deltaY * 0.002))));
  }, []);

  useEffect(() => {
    window.addEventListener("mousemove", onMoveG);
    window.addEventListener("mouseup", onUpG);
    return () => { window.removeEventListener("mousemove", onMoveG); window.removeEventListener("mouseup", onUpG); };
  }, [onMoveG, onUpG]);

  return (
    <div ref={containerRef} onMouseDown={onDown} onWheel={onWheelG}
      style={{ width: "100%", height: "100%", overflow: "hidden", cursor: "grab", position: "relative", background: "#0b0e14" }}>
      <svg width="100%" height="100%" style={{ position: "absolute", top: 0, left: 0 }}>
        <g transform={`translate(${pan.x},${pan.y}) scale(${zoom})`}>
          {layout.edges.map((e, i) => {
            const jc = JOINT_COLORS[e.joint.type] || "#6b7280";
            const midY = (e.from.y + e.to.y) / 2 + 14;
            return (
              <g key={i}>
                <path
                  d={`M${e.from.x},${e.from.y + 14} C${e.from.x},${midY} ${e.to.x},${midY} ${e.to.x},${e.to.y - 14}`}
                  fill="none" stroke={jc} strokeWidth={1.5} strokeOpacity={0.5}
                />
                <circle cx={(e.from.x + e.to.x) / 2} cy={midY - 4} r={4} fill={jc} fillOpacity={0.4} />
                <title>{e.joint.name} ({e.joint.type})</title>
              </g>
            );
          })}
          {layout.nodes.map((n) => {
            const sel = n.name === selectedLink;
            const w = Math.max(n.name.length * 7.2 + 24, 64);
            return (
              <g key={n.name} className="graph-node" style={{ cursor: "pointer" }} onClick={() => onSelectLink(n.name)}>
                <rect
                  x={n.x - w / 2} y={n.y - 14} width={w} height={28} rx={4}
                  fill={sel ? "#1e3a5f" : "#151c28"}
                  stroke={sel ? "#38bdf8" : n.hasVis ? "#2a3444" : "#1a2030"}
                  strokeWidth={sel ? 1.5 : 1}
                />
                {n.hasVis && <rect x={n.x - w / 2 + 7} y={n.y - 3} width={5} height={5} rx={1} fill="#38bdf8" fillOpacity={0.6} />}
                <text x={n.x + (n.hasVis ? 4 : 0)} y={n.y + 3.5} textAnchor="middle" fontSize={9.5}
                  fill={sel ? "#e2e8f0" : "#94a3b8"} fontFamily="'JetBrains Mono', monospace" fontWeight={sel ? 600 : 400}>
                  {n.name}
                </text>
              </g>
            );
          })}
        </g>
      </svg>
      {layout.nodes.length === 0 && (
        <div style={{ position: "absolute", top: "50%", left: "50%", transform: "translate(-50%,-50%)", fontSize: 11, color: "#475569" }}>No links</div>
      )}
    </div>
  );
}

// ─── Main App (3-column: left panel | viewport | graph + code) ─
export default function URDFEditor() {
  const [code, setCode] = useState(DEFAULT_URDF);
  const [parsedData, setParsedData] = useState(null);
  const [selectedLink, setSelectedLink] = useState(null);
  const [tree, setTree] = useState([]);
  const [jointValues, setJointValues] = useState({});
  const [showAddLink, setShowAddLink] = useState(false);
  const [showAddJoint, setShowAddJoint] = useState(false);
  const [activeTab, setActiveTab] = useState("tree");
  const [leftWidth, setLeftWidth] = useState(280);
  const [rightWidth, setRightWidth] = useState(380);
  const [graphHeight, setGraphHeight] = useState(260);
  const isDragL = useRef(false);
  const isDragR = useRef(false);
  const isDragG = useRef(false);

  useEffect(() => {
    const parsed = parseURDF(code);
    setParsedData(parsed);
    if (!parsed.error) setTree(buildTree(parsed.links, parsed.joints));
  }, [code]);

  useEffect(() => {
    const onMove = (e) => {
      if (isDragL.current) setLeftWidth(Math.max(220, Math.min(450, e.clientX)));
      if (isDragR.current) setRightWidth(Math.max(280, Math.min(600, window.innerWidth - e.clientX)));
      if (isDragG.current) {
        const rp = document.getElementById("right-col");
        if (rp) {
          const rect = rp.getBoundingClientRect();
          setGraphHeight(Math.max(120, Math.min(rect.height - 150, e.clientY - rect.top - 28)));
        }
      }
    };
    const onUp = () => {
      isDragL.current = false; isDragR.current = false; isDragG.current = false;
      document.body.style.cursor = "default"; document.body.style.userSelect = "auto";
    };
    window.addEventListener("mousemove", onMove);
    window.addEventListener("mouseup", onUp);
    return () => { window.removeEventListener("mousemove", onMove); window.removeEventListener("mouseup", onUp); };
  }, []);

  const addSnippet = (xml) => { const p = code.lastIndexOf("</robot>"); if (p >= 0) setCode(code.slice(0, p) + xml + "\n" + code.slice(p)); setShowAddLink(false); setShowAddJoint(false); };
  const selectedLinkData = parsedData?.links?.find((l) => l.name === selectedLink);
  const selectedJoints = parsedData?.joints?.filter((j) => j.parent === selectedLink || j.child === selectedLink) || [];
  const movableJoints = parsedData?.joints?.filter((j) => j.type === "revolute" || j.type === "continuous" || j.type === "prismatic") || [];

  const startDrag = (ref) => () => { ref.current = true; document.body.style.cursor = "col-resize"; document.body.style.userSelect = "none"; };
  const startDragV = () => { isDragG.current = true; document.body.style.cursor = "row-resize"; document.body.style.userSelect = "none"; };

  return (
    <div style={{ width: "100vw", height: "100vh", display: "flex", background: "#0b0e14", color: "#e2e8f0", fontFamily: "'JetBrains Mono', 'Fira Code', 'SF Mono', monospace", overflow: "hidden", position: "relative" }}>
      <link href="https://fonts.googleapis.com/css2?family=JetBrains+Mono:wght@300;400;500;600&display=swap" rel="stylesheet" />
      {showAddLink && <AddLinkDialog onAdd={addSnippet} onClose={() => setShowAddLink(false)} />}
      {showAddJoint && parsedData && <AddJointDialog links={parsedData.links} onAdd={addSnippet} onClose={() => setShowAddJoint(false)} />}

      {/* ═══ LEFT PANEL ═══ */}
      <div style={{ width: leftWidth, flexShrink: 0, display: "flex", flexDirection: "column", borderRight: "1px solid #1e293b", background: "#0f1219" }}>
        <div style={{ padding: "14px 16px 10px", borderBottom: "1px solid #1e293b" }}>
          <div style={{ display: "flex", alignItems: "center", gap: 8, marginBottom: 6 }}>
            <span style={{ fontSize: 16 }}>⚙</span>
            <span style={{ fontSize: 13, fontWeight: 600, letterSpacing: 0.5 }}>URDF Editor</span>
          </div>
          {parsedData && !parsedData.error && <div style={{ fontSize: 11, color: "#64748b" }}>{parsedData.name} — {parsedData.links.length} links, {parsedData.joints.length} joints</div>}
          {parsedData?.error && <div style={{ fontSize: 11, color: "#ef4444", marginTop: 4, lineHeight: 1.3 }}>⚠ {parsedData.error.slice(0, 120)}</div>}
        </div>

        <div style={{ display: "flex", borderBottom: "1px solid #1e293b" }}>
          {[["tree", "Structure"], ["joints", "Joints"], ["info", "Info"]].map(([key, label]) => (
            <button key={key} onClick={() => setActiveTab(key)} style={{ flex: 1, padding: "8px 0", background: "transparent", border: "none", borderBottom: activeTab === key ? "2px solid #38bdf8" : "2px solid transparent", color: activeTab === key ? "#e2e8f0" : "#475569", fontSize: 11, cursor: "pointer", fontFamily: "inherit", letterSpacing: 0.5 }}>{label}</button>
          ))}
        </div>

        <div style={{ flex: 1, overflow: "auto" }}>
          {activeTab === "tree" && (
            <div style={{ paddingTop: 4 }}>
              {tree.map((root, i) => <TreeNode key={i} node={root} selectedLink={selectedLink} onSelectLink={setSelectedLink} />)}
              {tree.length === 0 && <div style={{ padding: 16, fontSize: 11, color: "#475569", textAlign: "center" }}>No links defined</div>}
            </div>
          )}
          {activeTab === "joints" && (
            <div style={{ padding: 8 }}>
              {movableJoints.length === 0 && <div style={{ padding: 16, fontSize: 11, color: "#475569", textAlign: "center" }}>No movable joints</div>}
              {movableJoints.map((j) => {
                const min = j.limit?.lower ?? -Math.PI; const max = j.limit?.upper ?? Math.PI; const val = jointValues[j.name] ?? 0;
                return (
                  <div key={j.name} style={{ marginBottom: 12, padding: "8px", background: "#0b0e14", borderRadius: 4 }}>
                    <div style={{ display: "flex", alignItems: "center", gap: 6, marginBottom: 6 }}>
                      <span style={{ color: JOINT_COLORS[j.type], fontSize: 12 }}>{JOINT_ICONS[j.type]}</span>
                      <span style={{ fontSize: 11, color: "#94a3b8", flex: 1 }}>{j.name}</span>
                      <span style={{ fontSize: 10, color: "#475569", fontVariantNumeric: "tabular-nums" }}>{val.toFixed(2)}</span>
                    </div>
                    <input type="range" min={min} max={max} step={0.01} value={val} onChange={(e) => setJointValues({ ...jointValues, [j.name]: parseFloat(e.target.value) })} style={{ width: "100%", accentColor: JOINT_COLORS[j.type] }} />
                    <div style={{ display: "flex", justifyContent: "space-between", fontSize: 9, color: "#334155", marginTop: 2 }}><span>{min.toFixed(2)}</span><span>{max.toFixed(2)}</span></div>
                  </div>
                );
              })}
              {movableJoints.length > 0 && <button onClick={() => setJointValues({})} style={{ width: "100%", padding: "6px 0", background: "#1e293b", border: "none", borderRadius: 4, color: "#64748b", fontSize: 11, cursor: "pointer", fontFamily: "inherit", marginTop: 4 }}>Reset All Joints</button>}
            </div>
          )}
          {activeTab === "info" && selectedLinkData && (
            <div style={{ padding: 12 }}>
              <div style={{ fontSize: 13, color: "#e2e8f0", marginBottom: 12, fontWeight: 600 }}>{selectedLinkData.name}</div>
              {selectedLinkData.visuals.length > 0 ? (
                <div style={{ marginBottom: 12 }}>
                  <div style={{ fontSize: 10, color: "#64748b", textTransform: "uppercase", letterSpacing: 1, marginBottom: 6 }}>Visuals ({selectedLinkData.visuals.length})</div>
                  {selectedLinkData.visuals.map((vis, vi) => (
                    <div key={vi} style={{ fontSize: 11, color: "#94a3b8", padding: 8, background: "#0b0e14", borderRadius: 4, marginBottom: 4 }}>
                      <div style={{ color: "#64748b", fontSize: 10, marginBottom: 2 }}>Visual #{vi + 1}</div>
                      <div>Type: {vis.geometry?.type || "none"}</div>
                      {vis.geometry?.size && <div>Size: {vis.geometry.size.join(" × ")}</div>}
                      {vis.geometry?.radius != null && <div>Radius: {vis.geometry.radius}</div>}
                      {vis.geometry?.length != null && <div>Length: {vis.geometry.length}</div>}
                    </div>
                  ))}
                </div>
              ) : <div style={{ fontSize: 11, color: "#475569", marginBottom: 12 }}>No visual geometry (virtual link)</div>}
              {selectedJoints.length > 0 && (
                <div>
                  <div style={{ fontSize: 10, color: "#64748b", textTransform: "uppercase", letterSpacing: 1, marginBottom: 6 }}>Connected Joints</div>
                  {selectedJoints.map((j) => (
                    <div key={j.name} style={{ fontSize: 11, color: "#94a3b8", padding: 6, marginBottom: 4, background: "#0b0e14", borderRadius: 4, display: "flex", alignItems: "center", gap: 6 }}>
                      <span style={{ color: JOINT_COLORS[j.type] }}>{JOINT_ICONS[j.type]}</span><span>{j.name}</span><span style={{ fontSize: 9, color: "#475569" }}>({j.type})</span>
                    </div>
                  ))}
                </div>
              )}
            </div>
          )}
          {activeTab === "info" && !selectedLinkData && <div style={{ padding: 16, fontSize: 11, color: "#475569", textAlign: "center" }}>Select a link to view details</div>}
        </div>

        <div style={{ padding: 8, borderTop: "1px solid #1e293b", display: "flex", gap: 6 }}>
          <button onClick={() => setShowAddLink(true)} style={{ flex: 1, padding: "7px 0", background: "transparent", border: "1px solid #1e293b", borderRadius: 4, color: "#38bdf8", fontSize: 11, cursor: "pointer", fontFamily: "inherit" }}
            onMouseEnter={(e) => { e.currentTarget.style.background = "#1e3a5f22"; e.currentTarget.style.borderColor = "#38bdf8"; }}
            onMouseLeave={(e) => { e.currentTarget.style.background = "transparent"; e.currentTarget.style.borderColor = "#1e293b"; }}>+ Link</button>
          <button onClick={() => setShowAddJoint(true)} style={{ flex: 1, padding: "7px 0", background: "transparent", border: "1px solid #1e293b", borderRadius: 4, color: "#f59e0b", fontSize: 11, cursor: "pointer", fontFamily: "inherit" }}
            onMouseEnter={(e) => { e.currentTarget.style.background = "#78350f22"; e.currentTarget.style.borderColor = "#f59e0b"; }}
            onMouseLeave={(e) => { e.currentTarget.style.background = "transparent"; e.currentTarget.style.borderColor = "#1e293b"; }}>+ Joint</button>
        </div>
      </div>

      {/* Left resize handle */}
      <div onMouseDown={startDrag(isDragL)} style={{ width: 4, cursor: "col-resize", background: "transparent", flexShrink: 0 }}
        onMouseEnter={(e) => (e.currentTarget.style.background = "#1e3a5f")} onMouseLeave={(e) => (e.currentTarget.style.background = "transparent")} />

      {/* ═══ CENTER: 3D VIEWPORT ═══ */}
      <div style={{ flex: 1, position: "relative", minWidth: 0 }}>
        <Viewport parsedData={parsedData} selectedLink={selectedLink} onSelectLink={setSelectedLink} jointValues={jointValues} />
        <div style={{ position: "absolute", top: 10, left: 12, fontSize: 10, color: "#334155", lineHeight: 1.6, pointerEvents: "none" }}>LMB: Orbit | RMB: Pan | Scroll: Zoom</div>
        <div style={{ position: "absolute", top: 10, right: 12, display: "flex", gap: 6, pointerEvents: "none" }}>
          {Object.entries(JOINT_COLORS).map(([type, color]) => (
            <div key={type} style={{ display: "flex", alignItems: "center", gap: 3, fontSize: 9, color: "#475569" }}>
              <span style={{ width: 7, height: 7, borderRadius: "50%", background: color, display: "inline-block" }} />{type}
            </div>
          ))}
        </div>
      </div>

      {/* Right resize handle */}
      <div onMouseDown={startDrag(isDragR)} style={{ width: 4, cursor: "col-resize", background: "transparent", flexShrink: 0 }}
        onMouseEnter={(e) => (e.currentTarget.style.background = "#1e3a5f")} onMouseLeave={(e) => (e.currentTarget.style.background = "transparent")} />

      {/* ═══ RIGHT PANEL: Graph + Code ═══ */}
      <div id="right-col" style={{ width: rightWidth, flexShrink: 0, display: "flex", flexDirection: "column", borderLeft: "1px solid #1e293b", background: "#0f1219" }}>

        {/* Connection Graph */}
        <div style={{ height: 28, display: "flex", alignItems: "center", padding: "0 12px", borderBottom: "1px solid #1e293b", background: "#0d1017", gap: 8, flexShrink: 0 }}>
          <span style={{ fontSize: 11, color: "#64748b" }}>⬡</span>
          <span style={{ fontSize: 11, color: "#64748b" }}>Connection Graph</span>
          <div style={{ flex: 1 }} />
          <span style={{ fontSize: 10, color: "#334155" }}>drag to pan · scroll to zoom</span>
        </div>
        <div style={{ height: graphHeight, flexShrink: 0 }}>
          <ConnectionGraph parsedData={parsedData} selectedLink={selectedLink} onSelectLink={setSelectedLink} />
        </div>

        {/* Graph/Code vertical resize handle */}
        <div onMouseDown={startDragV} style={{ height: 4, cursor: "row-resize", background: "transparent", borderTop: "1px solid #1e293b", flexShrink: 0 }}
          onMouseEnter={(e) => (e.currentTarget.style.background = "#1e3a5f")} onMouseLeave={(e) => (e.currentTarget.style.background = "transparent")} />

        {/* Code Editor */}
        <div style={{ flex: 1, display: "flex", flexDirection: "column", minHeight: 0, borderTop: "1px solid #1e293b" }}>
          <div style={{ height: 28, display: "flex", alignItems: "center", padding: "0 12px", borderBottom: "1px solid #1e293b", background: "#0d1017", gap: 8, flexShrink: 0 }}>
            <span style={{ fontSize: 10, color: "#38bdf8" }}>{"<>"}</span>
            <span style={{ fontSize: 11, color: "#64748b" }}>URDF Source</span>
            <div style={{ flex: 1 }} />
            <span style={{ fontSize: 10, color: "#334155" }}>{code.split("\n").length} lines</span>
          </div>
          <div style={{ flex: 1, minHeight: 0 }}>
            <CodeEditor code={code} onChange={setCode} />
          </div>
        </div>
      </div>
    </div>
  );
}
