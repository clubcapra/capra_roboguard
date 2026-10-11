'use strict';
const $ = id => document.getElementById(id);
let state = null, poll = null, activeConfig = null, refreshing = false;
let activeSensor = null, activeInfo = null, sensorHealth = null, commandInputs = [];
const policyKeys = ['max_attempts', 'attempt_timeout_ms', 'retry_delay_ms', 'stale_after_ms'];
const elem = (tag, text, cls) => { const e = document.createElement(tag); if (text !== undefined) e.textContent = text; if (cls) e.className = cls; return e; };
function notice(text, source = 'action') { $('message').textContent = text; $('message').dataset.source = source; $('message').hidden = false; }
function clearNotice() { $('message').textContent = ''; $('message').hidden = true; delete $('message').dataset.source; }
async function api(path, method = 'GET', body) {
  if (new URL(path, location.href).origin !== location.origin) throw new Error('Use a local HALo API path.');
  const options = { method, credentials: 'same-origin', headers: { 'X-HALo-Request': '1' }, cache: 'no-store' };
  if (body !== undefined) { options.headers['Content-Type'] = 'application/json'; options.body = JSON.stringify(body); }
  const r = await fetch(path, options);
  const text = await r.text(); let value; try { value = JSON.parse(text); } catch { value = text; }
  if (!r.ok) { if (r.status === 401 && path !== '/auth/login') showLogin(); throw new Error(value.error || `${r.status}: ${text}`); }
  return value;
}
function showLogin() { clearInterval(poll); poll = null; $('console').hidden = true; $('logout').hidden = true; $('login-panel').hidden = false; }
async function showConsole() { clearNotice(); $('console').hidden = false; $('logout').hidden = false; $('login-panel').hidden = true; await refresh(); clearInterval(poll); poll = setInterval(refresh, 3000); }
$('login-form').addEventListener('submit', async e => { e.preventDefault(); const button = e.target.querySelector('button'); button.disabled = true; try { await api('/auth/login', 'POST', { password: $('password').value }); $('password').value = ''; $('login-error').textContent = ''; await showConsole(); } catch (error) { $('login-error').textContent = error.message; } finally { button.disabled = false; } });
$('logout').onclick = async () => { try { await api('/auth/logout', 'POST'); showLogin(); } catch (e) { notice(e.message); } };
document.querySelectorAll('[data-view]').forEach(button => button.onclick = async () => {
  document.querySelectorAll('[data-view]').forEach(b => b.removeAttribute('aria-current')); button.setAttribute('aria-current', 'page');
  ['hardware', 'commands', 'connections', 'settings', 'api', 'logs'].forEach(v => $('view-' + v).hidden = v !== button.dataset.view);
  try { if (button.dataset.view === 'commands') await sensorTabs(); if (button.dataset.view === 'api') await reference(); if (button.dataset.view === 'logs') await logs(); } catch (e) { notice(e.message); }
});
function cell(row, child) { const td = elem('td'); td.append(typeof child === 'string' ? document.createTextNode(child) : child); row.append(td); }
function action(text, fn) { const b = elem('button', text); b.type = 'button'; b.onclick = async () => { b.disabled = true; try { await fn(); } catch (e) { notice(e.message); } finally { b.disabled = false; } }; return b; }
async function refresh() {
  if (refreshing) return; refreshing = true;
  try {
    state = await api('/admin/state'); $('connection-state').textContent = 'Connected'; document.querySelector('.status-light').style.background = '#3f9749'; $('last-refresh').textContent = new Date().toLocaleTimeString();
    const health = await Promise.all(state.sensors.map(async s => { try { return [s.id, await api(`/${s.id}/health`)]; } catch (e) { return [s.id, { controller_connected: false, hardware_ready: false, telemetry: { errors: [e.message] } }]; } }));
    const healthMap = new Map(health); $('ready-count').textContent = health.filter(([, h]) => h.hardware_ready).length;
    $('error-count').textContent = state.drivers.filter(d => d.error || d.state === 'degraded' || d.state === 'failed').length;
    $('drivers').replaceChildren(); $('sim-note').hidden = state.backend !== 'simulator';
    for (const d of state.drivers) {
      const row = elem('tr'); cell(row, d.id); cell(row, elem('span', d.state, 'badge ' + d.state)); cell(row, `${d.attempt} / ${d.policy.max_attempts}`);
      const errors = [d.error, ...Object.entries(d.sensors).flatMap(([id, h]) => h.errors.map(error => `${id}: ${error}`))].filter(Boolean);
      if (!errors.length && d.last_error) errors.push('Previous: ' + d.last_error);
      cell(row, elem('div', errors.join('\n') || 'No reported errors', 'diagnostics'));
      const controls = elem('div', undefined, 'actions'); const retry = action('Retry', async () => { await api(`/admin/drivers/${d.id}/retry`, 'POST'); notice(`Recovery scheduled for ${d.id}.`); await refresh(); });
      retry.disabled = ['connecting', 'retrying', 'stopping', 'timed_out'].includes(d.state); controls.append(retry, action('Configure', () => configure(d.id))); cell(row, controls); $('drivers').append(row);
    }
    $('devices').replaceChildren();
    for (const s of state.sensors) {
      const h = healthMap.get(s.id); const row = elem('tr'); cell(row, s.id); cell(row, h.controller_connected ? 'Connected' : 'Disconnected'); cell(row, h.hardware_ready ? 'Ready' : 'Unavailable');
      cell(row, elem('div', (h.telemetry.errors || []).join('\n') || (h.telemetry.stale ? 'Telemetry stale' : 'No reported errors'), 'diagnostics'));
      cell(row, action('Inspect', async () => { const data = await api(`/${s.id}/data`); $('request-path').value = `/${s.id}/data`; $('request-result').textContent = JSON.stringify(data, null, 2); document.querySelector('[data-view=api]').click(); })); $('devices').append(row);
    }
    if (!document.activeElement.closest('#ports') && !$('config-dialog').open) renderPorts();
    if (!$('view-commands').hidden) await sensorTabs();
    $('config-buttons').replaceChildren(...state.drivers.map(d => action(d.id, () => configure(d.id))));
    if ($('message').dataset.source === 'connection') clearNotice();
  } catch (e) { $('connection-state').textContent = 'Connection lost'; document.querySelector('.status-light').style.background = '#bd3b31'; notice(e.message, 'connection'); } finally { refreshing = false; }
}
function renderPorts() {
  $('ports').replaceChildren();
  for (const [id, ports] of Object.entries(state.ports)) {
    const row = elem('tr'); cell(row, id);
    const data = elem('input'), command = elem('input'); [data, command].forEach(i => { i.type = 'number'; i.min = '1'; i.max = '65535'; }); data.value = ports[0]; command.value = ports[1]; data.setAttribute('aria-label', `${id} data port`); command.setAttribute('aria-label', `${id} command port`);
    cell(row, data); cell(row, command); cell(row, action('Save ports', async () => { await api(`/admin/sensors/${id}/ports`, 'POST', { data_port: Number(data.value), command_port: Number(command.value) }); notice(`Ports saved for ${id}. Refresh discovery in connected software.`); await refresh(); })); $('ports').append(row);
  }
}
async function configure(id) {
  activeConfig = id; const d = state.drivers.find(d => d.id === id); const config = d.configuration_endpoint ? await api(d.configuration_endpoint) : {text: 'Connection settings are supplied by this backend. Recovery policy remains configurable.'};
  $('config-title').textContent = `${id} configuration`; $('config-text').disabled = !d.configuration_endpoint; $('config-form').querySelector('button[type=submit]').disabled = !d.configuration_endpoint; $('config-text').value = config.text; $('config-error').textContent = '';
  policyKeys.forEach(k => $('policy-' + k).value = d.policy[k]); $('config-dialog').showModal();
}
$('close-config').onclick = () => $('config-dialog').close();
$('config-form').onsubmit = async e => { e.preventDefault(); try { await api(state.drivers.find(d => d.id === activeConfig).configuration_endpoint, 'POST', { text: $('config-text').value }); $('config-dialog').close(); notice(`Connection saved for ${activeConfig}; recovery scheduled.`); await refresh(); } catch (e) { $('config-error').textContent = e.message; } };
$('save-policy').onclick = async () => { try { const policy = Object.fromEntries(policyKeys.map(k => [k, Number($('policy-' + k).value)])); await api(`/admin/drivers/${activeConfig}/policy`, 'POST', policy); $('config-error').textContent = 'Recovery policy saved.'; } catch (e) { $('config-error').textContent = e.message; } };
$('restart').onclick = async () => { if (!confirm('Restart HALo and reconnect all hardware? Active UDP subscriptions will need to reconnect.')) return; try { await api('/admin/reload', 'POST'); notice('Restart scheduled. Sign in again when HALo is ready.'); } catch (e) { notice(e.message); } };
async function reference() {
  const [manifest, spec] = await Promise.all([api('/discover'), api('/openapi.json')]); $('manifest').textContent = JSON.stringify(manifest, null, 2); $('operations').replaceChildren();
  for (const [path, methods] of Object.entries(spec.paths)) for (const [method, op] of Object.entries(methods)) {
    if (!['get', 'post', 'put', 'delete', 'patch'].includes(method)) continue;
    const detail = elem('details'), summary = elem('summary'); summary.append(elem('b', method.toUpperCase()), document.createTextNode(`${path} · ${op.summary || ''}`)); detail.append(summary, elem('p', op.description || ''), elem('pre', JSON.stringify(op, null, 2)), action('Use request', async () => { $('request-method').value = method.toUpperCase(); $('request-path').value = path; })); $('operations').append(detail);
  }
}
$('request-form').onsubmit = async e => { e.preventDefault(); try { const path = $('request-path').value; if (!path.startsWith('/') || path.startsWith('//')) throw new Error('Use a local API path starting with /.'); const method = $('request-method').value; $('request-result').textContent = JSON.stringify(await api(path, method, method === 'GET' ? undefined : JSON.parse($('request-body').value)), null, 2); } catch (e) { $('request-result').textContent = e.message; } };
async function logs() {
  const index = await api('/logs'); $('logs').replaceChildren();
  for (const day of index.days || []) for (const hour of day.hours || []) {
    const detail = elem('details'); detail.append(elem('summary', `${day.date} ${hour.hour}:00 UTC`));
    for (const file of [...(hour.sensors || []).map(id => id + '.csv'), ...(hour.has_inputs || hour.inputs ? ['inputs.csv'] : [])]) { const link = elem('a', file); link.href = `/logs/file/${day.date}/${hour.hour}/${file}`; link.download = file; const p = elem('p'); p.append(link); detail.append(p); } $('logs').append(detail);
  }
  if (!$('logs').children.length) $('logs').textContent = 'No logs available yet.';
}
(async () => { try { const status = await api('/auth/status'); if (status.authenticated) await showConsole(); else { showLogin(); if (!status.configured) $('login-error').textContent = 'Set a password locally with halo --set-password, then restart HALo.'; } } catch (e) { $('login-error').textContent = e.message; } })();


function fieldInput(field) {
  let input;
  if (field.type_name === 'bool') {
    input = elem('select');
    for (const [value, title] of [['', 'Omit'], ['true', 'True'], ['false', 'False']]) { const option = elem('option', title); option.value = value; input.append(option); }
  } else if (/^[ufi]\d+$/.test(field.type_name)) {
    input = elem('input'); input.type = 'number'; input.step = field.type_name.startsWith('f') ? 'any' : '1';
    const match = field.type_name.match(/^([ui])(\d+)$/);
    if (match) { const bits = Number(match[2]); input.min = match[1] === 'u' ? '0' : String(Math.max(-(2 ** (bits - 1)), -Number.MAX_SAFE_INTEGER)); input.max = String(Math.min(2 ** (bits - (match[1] === 'i' ? 1 : 0)) - 1, Number.MAX_SAFE_INTEGER)); }
    input.placeholder = 'Omit';
  } else { input = elem(field.type_name === 'String' || field.type_name === 'str' ? 'input' : 'textarea'); input.placeholder = input.tagName === 'TEXTAREA' ? 'JSON, or leave blank to omit' : 'Omit'; }
  input.setAttribute('aria-label', field.name); input.dataset.field = field.name;
  return input;
}
function fieldValue(field, input) {
  if (input.value.trim() === '') return undefined;
  if (field.type_name === 'bool') return input.value === 'true';
  if (/^[ufi]\d+$/.test(field.type_name)) {
    const value = Number(input.value);
    if (!Number.isFinite(value) || (/^[ui]/.test(field.type_name) && !Number.isSafeInteger(value))) throw new Error(`${field.name}: enter a valid ${field.type_name}.`);
    return value;
  }
  if (field.type_name === 'String' || field.type_name === 'str') return input.value;
  return JSON.parse(input.value);
}
function commandPayload() {
  const payload = {};
  for (const {field, input} of commandInputs) { const value = fieldValue(field, input); if (value !== undefined) payload[field.name] = value; }
  if (!Object.keys(payload).length) throw new Error('Select at least one command field.');
  for (const {field} of commandInputs) if (Object.hasOwn(payload, field.name)) {
    if (field.standalone && Object.keys(payload).length !== 1) throw new Error(`${field.name} must be sent alone.`);
    if ((field.requires_ready || (field.ready_value !== undefined && JSON.stringify(payload[field.name]) === JSON.stringify(field.ready_value))) && !sensorHealth?.hardware_ready) throw new Error('Hardware is not ready for the selected command.');
  }
  return payload;
}
async function sensorTabs() {
  const manifest = await api('/discover');
  const available = new Set(manifest.sensors.map(s => s.id));
  const ids = state.sensors.map(s => s.id);
  if (JSON.stringify(ids) !== $('sensor-tabs').dataset.identities) {
    $('sensor-tabs').dataset.identities = JSON.stringify(ids); $('sensor-tabs').replaceChildren();
    for (const sensor of state.sensors) {
      const tab = action(sensor.id, () => selectSensor(sensor.id)); tab.setAttribute('role', 'tab'); tab.dataset.sensor = sensor.id; tab.setAttribute('aria-controls', 'sensor-panel'); $('sensor-tabs').append(tab);
    }
  }
  for (const tab of $('sensor-tabs').children) tab.title = available.has(tab.dataset.sensor) ? 'Available in discovery' : 'Hardware unavailable';
  if (!ids.includes(activeSensor)) { activeSensor = null; activeInfo = null; if (ids.length) await selectSensor(ids[0]); }
  else await sensorFeedback();
}
async function selectSensor(id) {
  activeSensor = id; activeInfo = null; sensorHealth = null; commandInputs = [];
  $('sensor-command-send').hidden = true; $('sensor-estop').hidden = true; $('sensor-command-fields').replaceChildren(elem('p', 'Loading driver schema…'));
  for (const tab of $('sensor-tabs').children) { const selected = tab.dataset.sensor === id; tab.setAttribute('aria-selected', String(selected)); if (selected) tab.id = 'sensor-tab-active'; else tab.removeAttribute('id'); }
  const info = await api(`/${encodeURIComponent(id)}/info`);
  if (id !== activeSensor) return;
  activeInfo = info; $('sensor-estop').hidden = !info.has_estop;
  $('sensor-title').textContent = `${activeInfo.display_name} (${id})`;
  $('sensor-command-result').hidden = true; $('sensor-command-hint').textContent = ''; commandInputs = [];
  const fields = activeInfo.command_schema;
  $('sensor-command-send').hidden = !fields.length;
  if (!fields.length) $('sensor-command-fields').replaceChildren(elem('p', 'This driver declares no commands.'));
  else {
    const table = elem('table'), head = elem('thead'), headings = elem('tr');
    for (const title of ['Command field', 'Value', 'Unit', 'Description']) headings.append(elem('th', title));
    head.append(headings); table.append(head); const body = elem('tbody');
    for (const field of fields) { const row = elem('tr'), input = fieldInput(field); commandInputs.push({field,input}); cell(row, field.name); cell(row, input); cell(row, field.unit || ''); cell(row, field.description); body.append(row); }
    table.append(body); $('sensor-command-fields').replaceChildren(table);
  }
  const mode = activeInfo.command_mode;
  $('sensor-mode').textContent = mode.type === 'Stream' ? `Command cadence: ${mode.interval_ms} ms. This form sends one command.` : 'One-shot commands.';
  await sensorFeedback();
}
async function sensorFeedback() {
  if (!activeSensor || !activeInfo) return;
  const id = activeSensor;
  try {
    const [health, data] = await Promise.all([api(`/${id}/health`), api(`/${id}/data`)]);
    if (id !== activeSensor) return;
    sensorHealth = health; $('sensor-health').textContent = `${health.hardware_ready ? 'Hardware ready' : 'Hardware unavailable'} · ${health.controller_connected ? 'Controller connected' : 'Controller disconnected'}${health.telemetry.errors.length ? ' · ' + health.telemetry.errors.join('; ') : ''}`;
    $('sensor-data').replaceChildren();
    for (const field of activeInfo.data_schema) {
      const row = elem('tr'); cell(row, field.name);
      const value = data[field.name]; cell(row, typeof value === 'object' && value !== null ? elem('pre', JSON.stringify(value, null, 2)) : String(value ?? '—')); cell(row, field.unit || ''); cell(row, field.description); $('sensor-data').append(row);
    }
  } catch (error) { if (id !== activeSensor) return; sensorHealth = null; $('sensor-health').textContent = error.message; }
}
$('sensor-command-form').onsubmit = async event => {
  event.preventDefault(); const send = $('sensor-command-send'); send.disabled = true;
  const id = activeSensor;
  try {
    const payload = commandPayload();
    for (const {field} of commandInputs) if (Object.hasOwn(payload, field.name) && field.confirmation && (field.confirmation_value === undefined || JSON.stringify(field.confirmation_value) === JSON.stringify(payload[field.name]))) { if (!confirm(field.confirmation)) return; }
    const result = await api(`/admin/sensors/${encodeURIComponent(id)}/command`, 'POST', payload);
    if (id !== activeSensor) return;
    $('sensor-command-result').textContent = JSON.stringify(result, null, 2); $('sensor-command-result').hidden = false; $('sensor-command-hint').textContent = 'Command acknowledged. Check sensor feedback for hardware state.'; await sensorFeedback();
  } catch (error) { if (id === activeSensor) $('sensor-command-hint').textContent = error.message; }
  finally { send.disabled = false; }
};

$('password-form').onsubmit = async event => {
  event.preventDefault(); const button = event.target.querySelector('button'); button.disabled = true;
  try {
    if ($('new-password').value !== $('confirm-password').value) throw new Error('New passwords do not match.');
    await api('/auth/password', 'POST', { current_password: $('current-password').value, new_password: $('new-password').value });
    event.target.reset(); $('password-error').textContent = ''; showLogin(); $('login-error').textContent = 'Password changed. Sign in with the new password.';
  } catch (error) { $('password-error').textContent = error.message; }
  finally { button.disabled = false; }
};

$('sensor-estop').onclick = async () => {
  const id = activeSensor, info = activeInfo, button = $('sensor-estop');
  if (!info?.has_estop || !confirm(info.estop_confirmation || 'Emergency stop this device?')) return;
  button.disabled = true;
  try {
    const result = await api(`/admin/sensors/${encodeURIComponent(id)}/estop`, 'POST', {});
    if (id !== activeSensor) return;
    $('sensor-command-result').textContent = JSON.stringify(result, null, 2); $('sensor-command-result').hidden = false;
    $('sensor-command-hint').textContent = 'Emergency stop acknowledged. Check hardware feedback.'; await sensorFeedback();
  } catch (error) { if (id === activeSensor) $('sensor-command-hint').textContent = error.message; }
  finally { button.disabled = false; }
};
