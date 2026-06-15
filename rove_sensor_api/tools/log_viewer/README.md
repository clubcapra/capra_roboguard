# Rove log viewer

Modular graph viewer for the `rove_sensor_api` CSV logs and live UDP feed.

- **Historical** mode reads CSVs from `$LOG_DIR` (default `/var/log/rove-sensor-api`, matching the systemd unit).
- **Live** mode subscribes to the running API over UDP (same protocol as `sensor_dashboard.py`).
- Each card is independent: pick a sensor + fields, attach to the shared timeline or detach for a per-card window.
- Drag-to-zoom on any chart updates the relevant window (shared or local).

Built with Vite + React + TypeScript + Blueprint.js + uPlot.

## Run

```bash
./run.sh
```

That's it — `run.sh` bootstraps a local Node toolchain into `.node/` (no sudo, no apt) if `node` isn't installed, runs `npm install` if dependencies are missing, then starts the dev server.

Open <http://localhost:5173>. Vite proxies `/api` and `/ws` to the Node backend on `:8765`.

Other modes:

```bash
./run.sh build     # production build → dist/
./run.sh clean     # wipe .node/, node_modules/, dist/
```

### Env vars

| Var                  | Default                      | Notes                                      |
| -------------------- | ---------------------------- | ------------------------------------------ |
| `LOG_DIR`            | `/var/log/rove-sensor-api`   | Where to read historical CSVs from.        |
| `ROVE_API_BASE`      | `http://127.0.0.1:8080`      | Live API for `/discover` and UDP subscribe.|
| `LOG_VIEWER_API_PORT`| `8765`                       | Port for the Node backend.                 |

## Layout

```
tools/log_viewer/
├── server/         # Express + ws + UDP bridge
│   ├── csvReader.ts
│   ├── liveBridge.ts
│   └── index.ts
└── src/
    ├── components/
    │   ├── UPlotChart.tsx     # uPlot wrapper (canvas, fast)
    │   ├── Timeline.tsx       # reusable brush — give it bounds + value + onChange
    │   ├── SensorCard.tsx     # one card = one sensor; field multi-select; own/shared window
    │   └── AddSensorDialog.tsx
    ├── api.ts                 # REST + reconnecting WS client
    ├── state.ts               # zustand store
    └── App.tsx                # shell + shared timeline footer
```

## Modular use

- Use one card per sensor, all linked to the shared timeline footer — scrub once, every chart re-fetches.
- Or flip the **shared** switch on a card to give it its own timeline strip.
- `Timeline`, `UPlotChart`, and `SensorCard` have no implicit dependencies on each other — they share state through the zustand store only when you wire them that way.
