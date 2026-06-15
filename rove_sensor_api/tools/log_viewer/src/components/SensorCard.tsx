import { useEffect, useMemo, useRef, useState } from "react";
import { Button, Card as BPCard, Classes, MenuItem, Switch, Tag, Tooltip } from "@blueprintjs/core";
import { MultiSelect } from "@blueprintjs/select";
import { fetchRange, liveSocket, type LiveMessage } from "../api";
import { useStore } from "../state";
import type { Card as CardModel, RangePoint, SensorInfo } from "../types";
import { UPlotChart } from "./UPlotChart";
import { Timeline } from "./Timeline";

const LIVE_BUFFER_MS = 60_000;
const MAX_POINTS = 4000;

export function SensorCard({ card, sensor }: { card: CardModel; sensor: SensorInfo }) {
  const mode = useStore((s) => s.mode);
  const sharedWindow = useStore((s) => s.sharedWindow);
  const updateCard = useStore((s) => s.updateCard);
  const removeCard = useStore((s) => s.removeCard);

  const window =
    card.useSharedWindow || !card.localWindow
      ? sharedWindow
      : card.localWindow;

  // Plottable fields (numeric, excluding timestamp_ns).
  const plottable = useMemo(
    () => sensor.fields.filter((f) => f !== "timestamp_ns"),
    [sensor.fields],
  );
  const activeFields = card.fields.length ? card.fields : plottable.slice(0, 3);

  const [historicalPoints, setHistoricalPoints] = useState<RangePoint[]>([]);
  const [historicalFields, setHistoricalFields] = useState<string[]>(activeFields);
  const [loading, setLoading] = useState(false);
  const liveBufferRef = useRef<RangePoint[]>([]);
  const [liveTick, setLiveTick] = useState(0);

  // Historical fetch on window/field change.
  useEffect(() => {
    if (mode !== "historical") return;
    const ctrl = new AbortController();
    setLoading(true);
    fetchRange({
      sensor: sensor.id,
      start_ms: window.start_ms,
      end_ms: window.end_ms,
      fields: activeFields,
      maxPoints: MAX_POINTS,
      signal: ctrl.signal,
    })
      .then((r) => {
        setHistoricalPoints(r.points);
        setHistoricalFields(r.fields);
      })
      .catch(() => {})
      .finally(() => setLoading(false));
    return () => ctrl.abort();
  }, [mode, sensor.id, window.start_ms, window.end_ms, activeFields.join(",")]);

  // Live subscription.
  useEffect(() => {
    if (mode !== "live" || !sensor.live) return;
    liveSocket.connect();
    liveSocket.subscribe(sensor.id);
    const off = liveSocket.addListener((msg: LiveMessage) => {
      if (msg.type !== "data" || msg.sensor !== sensor.id) return;
      const values = activeFields.map((f) => {
        const v = msg.fields[f];
        if (typeof v === "number") return v;
        if (typeof v === "boolean") return v ? 1 : 0;
        return null;
      });
      const buf = liveBufferRef.current;
      buf.push({ t_ms: msg.t_ms, values });
      const cutoff = msg.t_ms - LIVE_BUFFER_MS;
      while (buf.length && buf[0].t_ms < cutoff) buf.shift();
      setLiveTick((t) => t + 1);
    });
    return () => {
      off();
      liveSocket.unsubscribe(sensor.id);
    };
  }, [mode, sensor.id, sensor.live, activeFields.join(",")]);

  const points = mode === "live" ? liveBufferRef.current : historicalPoints;
  const fieldList = mode === "live" ? activeFields : historicalFields;

  const data = useMemo<number[][]>(() => {
    const xs: number[] = new Array(points.length);
    const cols: number[][] = fieldList.map(() => new Array(points.length));
    for (let i = 0; i < points.length; i++) {
      xs[i] = points[i].t_ms;
      for (let j = 0; j < fieldList.length; j++) {
        const v = points[i].values[j];
        cols[j][i] = v == null ? NaN : v;
      }
    }
    return [xs, ...cols];
    // liveTick is a dep so live buffer mutations trigger a recompute.
  }, [points, fieldList.join(","), liveTick]);

  const liveWindow =
    mode === "live"
      ? { start_ms: (points.at(-1)?.t_ms ?? Date.now()) - LIVE_BUFFER_MS, end_ms: points.at(-1)?.t_ms ?? Date.now() }
      : window;

  return (
    <BPCard className="sensor-card" elevation={1}>
      <div className="sensor-card-header">
        <div>
          <h3 className={Classes.HEADING}>{sensor.display_name}</h3>
          <Tag minimal className="sensor-card-id">{sensor.id}</Tag>
          {sensor.live && <Tag minimal intent="success">live capable</Tag>}
          {loading && <Tag minimal intent="primary">loading…</Tag>}
        </div>
        <div className="sensor-card-actions">
          <Tooltip content={card.useSharedWindow ? "Detach from shared timeline" : "Attach to shared timeline"}>
            <Switch
              checked={card.useSharedWindow}
              label="shared"
              onChange={(e) => {
                const checked = e.currentTarget.checked;
                updateCard(card.uid, {
                  useSharedWindow: checked,
                  localWindow: checked ? undefined : { ...window },
                });
              }}
            />
          </Tooltip>
          <Button minimal icon="cross" onClick={() => removeCard(card.uid)} />
        </div>
      </div>

      <MultiSelect<string>
        items={plottable}
        selectedItems={activeFields}
        tagRenderer={(f) => f}
        itemRenderer={(f, { handleClick, modifiers }) => (
          <MenuItem
            key={f}
            text={f}
            active={modifiers.active}
            icon={activeFields.includes(f) ? "tick" : "blank"}
            onClick={handleClick}
            shouldDismissPopover={false}
          />
        )}
        onItemSelect={(f) => {
          const next = activeFields.includes(f) ? activeFields.filter((x) => x !== f) : [...activeFields, f];
          updateCard(card.uid, { fields: next });
        }}
        onRemove={(f) => updateCard(card.uid, { fields: activeFields.filter((x) => x !== f) })}
        itemPredicate={(query, item) => item.toLowerCase().includes(query.toLowerCase())}
        placeholder="fields to plot…"
        popoverProps={{ minimal: true }}
        resetOnSelect
      />

      <UPlotChart
        data={data}
        series={fieldList.map((f) => ({ label: f }))}
        height={240}
        window={liveWindow}
        onZoom={(a, b) => {
          if (mode === "live") return;
          if (card.useSharedWindow) {
            useStore.getState().setSharedWindow({ start_ms: a, end_ms: b });
          } else {
            updateCard(card.uid, { localWindow: { start_ms: a, end_ms: b } });
          }
        }}
      />

      {!card.useSharedWindow && mode === "historical" && (
        <Timeline
          bounds={{
            start_ms: dateMin(sensor.dates) ?? window.start_ms - 24 * 3_600_000,
            end_ms: Date.now(),
          }}
          value={window}
          onChange={(w) => updateCard(card.uid, { localWindow: w })}
          height={42}
          compact
        />
      )}
    </BPCard>
  );
}

function dateMin(dates: string[]): number | null {
  if (!dates.length) return null;
  return Date.parse(`${dates[0]}T00:00:00Z`);
}
