// Joint-pose panel — injected into the bundled ForgeBOT UI by index.html.
//
// The stock bundle only exposes the kinova "Sync" button; this adds a panel to
// directly ROTATE any movable joint (arm + flippers) so the operator can pose
// the model to the real robot's physical pose, then Sync. It talks only to the
// engine's HTTP API (/api/v1/joints, /api/v1/joints/set, /api/v1/flippers/sync);
// the 3D model follows because the engine re-broadcasts joint state over /state.
//
// Vanilla JS, no framework — it mounts its own fixed panel and never touches
// the React app's #root, so it can't break the viewport.
(function () {
  "use strict";
  const API = "";                 // same origin as the page (engine :9101)
  const POLL_MS = 500;
  let dragging = null;            // joint id currently being dragged (pause its refresh)

  async function getJSON(url) {
    const r = await fetch(API + url);
    if (!r.ok) throw new Error(url + " -> " + r.status);
    return r.json();
  }
  async function postJSON(url, body) {
    const r = await fetch(API + url, {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(body || {}),
    });
    return r.json().catch(() => ({}));
  }

  function el(tag, props, kids) {
    const e = document.createElement(tag);
    if (props) for (const k in props) {
      if (k === "style") Object.assign(e.style, props[k]);
      else if (k.startsWith("on")) e.addEventListener(k.slice(2).toLowerCase(), props[k]);
      else e.setAttribute(k, props[k]);
    }
    (kids || []).forEach((c) => e.appendChild(typeof c === "string" ? document.createTextNode(c) : c));
    return e;
  }

  // ---- panel chrome ----
  const panel = el("div", { style: {
    position: "fixed", top: "48px", left: "8px", zIndex: 9999, width: "270px",
    maxHeight: "calc(100vh - 64px)", overflowY: "auto",
    background: "rgba(9,9,11,0.92)", border: "1px solid #27272a", borderRadius: "8px",
    color: "#e4e4e7", font: "12px ui-monospace, monospace", padding: "10px",
    backdropFilter: "blur(4px)", boxShadow: "0 8px 24px rgba(0,0,0,0.5)",
  }});

  const header = el("div", { style: {
    display: "flex", justifyContent: "space-between", alignItems: "center",
    marginBottom: "8px", cursor: "pointer", userSelect: "none",
  }});
  let collapsed = false;
  const body = el("div");
  const title = el("div", { style: { fontWeight: "700", letterSpacing: "0.04em" } }, ["JOINT POSE"]);
  const chevron = el("span", { style: { color: "#71717a" } }, ["▼"]);
  header.appendChild(title); header.appendChild(chevron);
  header.addEventListener("click", () => {
    collapsed = !collapsed;
    body.style.display = collapsed ? "none" : "block";
    chevron.textContent = collapsed ? "▶" : "▼";
  });

  const status = el("div", { style: { color: "#a1a1aa", margin: "0 0 8px", minHeight: "14px" } }, [""]);

  const btnStyle = (bg, bd, fg) => ({
    flex: "1", padding: "6px", cursor: "pointer", background: bg, color: fg,
    border: "1px solid " + bd, borderRadius: "6px", font: "600 12px ui-monospace, monospace",
  });
  const syncBtn = el("button", { style: btnStyle("#155e57", "#0f766e", "#ecfeff"),
    title: "capture model<->ODrive offset for drums + flippers with a fresh frame",
    onclick: async () => {
      status.textContent = "syncing drives…";
      const r = await postJSON("/api/v1/flippers/sync", {});
      status.textContent = r.ok
        ? ("drives synced (" + r.captured + (r.missing_nodes && r.missing_nodes.length ? ", missing " + r.missing_nodes.join(",") : "") + ")")
        : ("drive sync: " + (r.errors ? r.errors.join("; ") : "no fresh frames"));
      refresh();
    }}, ["Sync drives to ODrive"]);
  const homeBtn = el("button", { style: btnStyle("#27272a", "#3f3f46", "#a1a1aa"),
    title: "snap all joints to the home pose (instant — no motion)",
    onclick: async () => {
      const r = await postJSON("/api/v1/joints/reset", {});
      status.textContent = "reset " + (r.reset || 0) + " joints to home (" + (r.source || "") + ")";
      refresh();
    }}, ["Reset to home"]);
  const setHomeBtn = el("button", { style: btnStyle("#3f3f46", "#52525b", "#e4e4e7"),
    title: "capture the current pose as the home pose (persisted)",
    onclick: async () => {
      const r = await postJSON("/api/v1/joints/home", {});
      status.textContent = "home set from current pose (" + (r.captured || 0) + " joints)";
    }}, ["Set home"]);
  const btnRow = el("div", { style: { display: "flex", gap: "6px", marginBottom: "6px" } }, [syncBtn]);
  const btnRow2 = el("div", { style: { display: "flex", gap: "6px", marginBottom: "6px" } }, [homeBtn, setHomeBtn]);

  // ---- pose library + pose-to-pose motion ----
  const inStyle = {
    background: "#18181b", color: "#e4e4e7", border: "1px solid #3f3f46",
    borderRadius: "4px", padding: "3px 4px", font: "12px ui-monospace, monospace",
  };
  const poseSel = el("select", { style: Object.assign({ flex: "1" }, inStyle) });
  const speedIn = el("input", { type: "number", min: "1", max: "180", value: "30",
    title: "move speed (deg/s of the fastest joint)", style: Object.assign({ width: "46px" }, inStyle) });
  const goBtn = el("button", { style: btnStyle("#155e57", "#0f766e", "#ecfeff"),
    title: "plan + run a smooth move to the selected pose (model only)",
    onclick: async () => {
      const name = poseSel.value;
      if (!name) { status.textContent = "no pose selected"; return; }
      const r = await postJSON("/api/v1/poses/goto", { name, speed_deg_s: Number(speedIn.value) || 30 });
      status.textContent = r.ok ? ("moving to " + name + " (" + r.duration_s + "s)") : ("goto: " + (r.error || "failed"));
    }}, ["Go"]);
  const stopBtn = el("button", { style: btnStyle("#7f1d1d", "#991b1b", "#fee2e2"),
    title: "abort the current move (joints hold)",
    onclick: async () => { await postJSON("/api/v1/poses/stop", {}); status.textContent = "motion stopped"; }}, ["Stop"]);
  const saveAsBtn = el("button", { style: btnStyle("#3f3f46", "#52525b", "#e4e4e7"),
    title: "save the current pose under a name",
    onclick: async () => {
      const name = (window.prompt("Save current pose as:") || "").trim();
      if (!name) return;
      const r = await postJSON("/api/v1/poses/save", { name });
      status.textContent = r.ok ? ("saved pose '" + name + "'") : ("save: " + (r.error || "failed"));
      loadPoses();
    }}, ["Save as…"]);
  const poseRow = el("div", { style: { display: "flex", gap: "6px", alignItems: "center", marginBottom: "6px" } },
    [poseSel, speedIn, goBtn, stopBtn]);
  const btnRow3 = el("div", { style: { display: "flex", gap: "6px", marginBottom: "10px" } }, [saveAsBtn]);

  // ---- flipper drive (normalised +1/0/-1 step, hold to ramp) ----
  function flipperHold(joint, step, glyph) {
    const b = el("button", { style: btnStyle("#1f2937", "#374151", "#e5e7eb"),
      title: joint + " " + (step > 0 ? "up" : "down") + " (hold)" }, [glyph]);
    const start = (e) => { e.preventDefault(); postJSON("/api/v1/flippers/command", { joint, step }); };
    const stop = () => postJSON("/api/v1/flippers/command", { joint, step: 0 });
    b.addEventListener("pointerdown", start);
    b.addEventListener("pointerup", stop);
    b.addEventListener("pointerleave", stop);
    return b;
  }
  function flipSign(joint) {
    const b = el("button", { style: btnStyle("#3f3f46", "#52525b", "#e4e4e7"),
      title: joint + ": flip motor↔model direction (re-anchored, persisted)" }, ["⇄"]);
    b.style.flex = "0 0 auto"; b.style.padding = "2px 8px";
    b.addEventListener("click", async () => {
      const r = await postJSON("/api/v1/flippers/sign", { joint });
      status.textContent = r.ok ? (joint + " sign → " + r.sign) : ("sign: " + (r.error || "failed"));
    });
    return b;
  }
  function flipperDriveRow(joint, label) {
    return el("div", { style: { display: "flex", gap: "6px", alignItems: "center", marginBottom: "4px" } }, [
      el("span", { style: { flex: "1", color: "#cbd5e1" } }, [label]),
      flipperHold(joint, +1, "▲"), flipperHold(joint, -1, "▼"), flipSign(joint),
    ]);
  }
  const flipSection = el("div", null, [
    el("div", { style: { color: "#71717a", margin: "6px 0 2px", fontSize: "10px", letterSpacing: "0.05em" } },
      ["FLIPPER DRIVE (hold ▲/▼ — +1/-1 step)"]),
    flipperDriveRow("FlipperFL", "Front-Left (41)"),
    flipperDriveRow("FlipperFR", "Front-Right (42)"),
  ]);

  let poseNames = "";
  async function loadPoses() {
    try {
      const d = await getJSON("/api/v1/poses");
      const names = (d.poses || []).map((p) => p.name);
      const key = names.join("|");
      if (key !== poseNames) {                 // rebuild only when the set changes
        poseNames = key;
        const cur = poseSel.value;
        poseSel.innerHTML = "";
        names.forEach((n) => poseSel.appendChild(el("option", { value: n }, [n])));
        if (names.includes(cur)) poseSel.value = cur;
      }
      if (d.motion && d.motion.active)
        status.textContent = "moving to " + d.motion.name + " — " + Math.round(d.motion.progress * 100) + "%";
    } catch (_) { /* engine busy */ }
  }

  const rows = el("div");
  body.appendChild(status);
  body.appendChild(btnRow);
  body.appendChild(btnRow2);
  body.appendChild(el("div", { style: { color: "#71717a", margin: "4px 0 2px", fontSize: "10px", letterSpacing: "0.05em" } }, ["POSES"]));
  body.appendChild(poseRow);
  body.appendChild(btnRow3);
  body.appendChild(flipSection);
  body.appendChild(rows);
  panel.appendChild(header);
  panel.appendChild(body);

  // ---- per-joint rows ----
  const rowByJoint = {};   // joint id -> {slider, num, badge}

  function makeRow(j) {
    const label = el("div", { style: { display: "flex", justifyContent: "space-between", marginBottom: "2px" } }, [
      el("span", { style: { color: "#fafafa" } }, [j.name || j.joint.slice(-8)]),
      el("span", { style: { color: j.mirrored ? "#fbbf24" : "#52525b", fontSize: "10px" } }, [j.mirrored ? "MIRRORED" : "free"]),
    ]);
    const slider = el("input", { type: "range", min: "-360", max: "360", step: "1",
      value: String(Math.round(j.angle_deg)), style: { flex: "1", accentColor: "#14b8a6" } });
    const num = el("input", { type: "number", step: "1", value: String(Math.round(j.angle_deg)),
      style: { width: "52px", background: "#18181b", color: "#e4e4e7", border: "1px solid #3f3f46",
               borderRadius: "4px", padding: "2px 4px", font: "12px ui-monospace, monospace" } });
    const zero = el("button", { style: { padding: "2px 6px", cursor: "pointer", background: "#27272a",
      color: "#a1a1aa", border: "1px solid #3f3f46", borderRadius: "4px" }, title: "set 0°" }, ["0"]);

    const send = async (deg) => {
      const v = Math.max(-360, Math.min(360, Number(deg) || 0));
      slider.value = String(v); num.value = String(v);
      await postJSON("/api/v1/joints/set", { joint: j.joint, angle_deg: v });
    };
    slider.addEventListener("pointerdown", () => { dragging = j.joint; });
    slider.addEventListener("pointerup", () => { dragging = null; });
    slider.addEventListener("input", () => { num.value = slider.value; send(slider.value); });
    num.addEventListener("change", () => send(num.value));
    zero.addEventListener("click", () => send(0));

    const ctl = el("div", { style: { display: "flex", gap: "4px", alignItems: "center", marginBottom: "8px" } },
      [slider, num, zero]);
    rowByJoint[j.joint] = { slider, num };
    return el("div", null, [label, ctl]);
  }

  async function build() {
    try {
      const { joints } = await getJSON("/api/v1/joints");
      rows.innerHTML = "";
      for (const k in rowByJoint) delete rowByJoint[k];
      // Named joints first (arm/flippers), drums/anonymous after.
      joints.sort((a, b) => (a.name || "zzz").localeCompare(b.name || "zzz"));
      joints.forEach((j) => rows.appendChild(makeRow(j)));
      status.textContent = joints.length + " joints — drag to pose, then Sync";
    } catch (e) {
      status.textContent = "engine not reachable: " + e.message;
    }
  }

  async function refresh() {
    try {
      const { joints } = await getJSON("/api/v1/joints");
      joints.forEach((j) => {
        const r = rowByJoint[j.joint];
        if (!r || dragging === j.joint) return;        // don't fight an active drag
        const v = Math.round(j.angle_deg);
        if (document.activeElement !== r.num) r.num.value = String(v);
        r.slider.value = String(v);
      });
    } catch (_) { /* transient */ }
  }

  function boot() {
    document.body.appendChild(panel);
    build().then(() => {
      loadPoses();
      setInterval(() => { refresh(); loadPoses(); }, POLL_MS);
    });
  }
  if (document.readyState === "loading") document.addEventListener("DOMContentLoaded", boot);
  else boot();
})();
