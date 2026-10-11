'use strict';
const $ = id => document.getElementById(id);
const keys = new Set();
const client = Array.from(crypto.getRandomValues(new Uint32Array(4)), x => x.toString(16)).join('');
let enabled = false, sequence = 0, pending = false;
async function request(path, body) {
  const response = await fetch(path, body === undefined ? {} : {
    method: 'POST', headers: {'Content-Type': 'application/json', 'X-Sim-Request': '1'},
    body: JSON.stringify(body), keepalive: true
  });
  if (!response.ok) throw new Error(await response.text());
  return response;
}
function values() {
  const forward = Number(keys.has('w')) - Number(keys.has('s'));
  const turn = Number(keys.has('d')) - Number(keys.has('a'));
  const scale = 0.25 / Math.max(1, Math.abs(forward + turn), Math.abs(forward - turn));
  return enabled ? [scale * (forward + turn), scale * (forward - turn)] : [0, 0];
}
function packet(left, right) { return {left, right, client, sequence: ++sequence}; }
function stop() {
  enabled = false;
  keys.clear();
  $('status').textContent = 'Disabled — stopped';
  $('tracks').textContent = 'Left 0.00 · Right 0.00';
  // This request bypasses a pending heartbeat; server sequence numbers reject late arrivals.
  request('/drive', packet(0, 0)).catch(() => {});
}
async function tick() {
  if (!enabled || pending) return;
  pending = true;
  const [left, right] = values();
  try {
    await request('/drive', packet(left, right));
    if (enabled) {
      $('status').textContent = 'Keyboard control enabled';
      $('tracks').textContent = `Left ${left.toFixed(2)} · Right ${right.toFixed(2)}`;
    }
  } catch (error) {
    stop();
    $('status').textContent = `Control stopped: ${error.message}`;
  } finally { pending = false; }
}
function signedIn() {
  $('login').hidden = true;
  $('console').hidden = false;
  $('view').src = '/stream.mjpg';
}
$('login').addEventListener('submit', async event => {
  event.preventDefault();
  try { await request('/login', {password: $('password').value}); $('password').value = ''; signedIn(); }
  catch (error) { $('error').textContent = error.message; }
});
$('logout').addEventListener('click', async () => {
  stop(); await request('/logout', {}); location.reload();
});
$('enable').addEventListener('click', () => {
  keys.clear(); enabled = true; tick();
});
$('stop').addEventListener('click', stop);
window.addEventListener('keydown', event => {
  if (event.target.matches('input,textarea') || !enabled) return;
  const key = event.key.toLowerCase();
  if (key === ' ') { event.preventDefault(); stop(); }
  if ('wasd'.includes(key) && key.length === 1) { event.preventDefault(); keys.add(key); tick(); }
});
window.addEventListener('keyup', event => {
  const key = event.key.toLowerCase();
  if (keys.delete(key)) {
    const [left, right] = values();
    request('/drive', packet(left, right)).catch(stop);
  }
});
window.addEventListener('blur', stop);
window.addEventListener('pagehide', stop);
document.addEventListener('visibilitychange', () => { if (document.hidden) stop(); });
setInterval(tick, 100);
request('/view').then(signedIn).catch(() => { $('login').hidden = false; });
