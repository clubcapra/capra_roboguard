import { useEffect, useMemo, useRef, useState } from "react";
import { Button, ButtonGroup, Tag } from "@blueprintjs/core";
import { UPlotChart } from "./UPlotChart";
import { DateRangeControl } from "./DateRangeControl";

export type TimelineProps = {
  /** Full available range (e.g. earliest→latest log). */
  bounds: { start_ms: number; end_ms: number };
  /** Currently selected window. */
  value: { start_ms: number; end_ms: number };
  onChange: (next: { start_ms: number; end_ms: number }) => void;
  /** Optional summary series rendered behind the brush. */
  summary?: { data: number[][]; label: string };
  /** Date strings (YYYY-MM-DD) with data — passed to the calendar for highlighting. */
  highlightDates?: string[];
  /** Show the calendar button in the controls bar. */
  showCalendar?: boolean;
  height?: number;
  compact?: boolean;
};

const PRESETS: { label: string; ms: number }[] = [
  { label: "1m", ms: 60_000 },
  { label: "10m", ms: 600_000 },
  { label: "1h", ms: 3_600_000 },
  { label: "6h", ms: 6 * 3_600_000 },
  { label: "1d", ms: 24 * 3_600_000 },
];

/** Timeline component: a brush over the full available range. Decoupled from any specific sensor —
 *  you give it bounds, a value, and a callback. Pluggable summary for visual context. */
export function Timeline({
  bounds,
  value,
  onChange,
  summary,
  highlightDates,
  showCalendar = true,
  height = 60,
  compact = false,
}: TimelineProps) {
  const ref = useRef<HTMLDivElement>(null);
  const [hover, setHover] = useState<number | null>(null);

  const data = useMemo<number[][]>(() => {
    if (summary && summary.data.length >= 2) return summary.data;
    // Empty placeholder series spanning bounds, so uPlot has an x-axis to draw.
    return [
      [bounds.start_ms, bounds.end_ms],
      [0, 0],
    ];
  }, [summary, bounds.start_ms, bounds.end_ms]);

  // Draw the brush region over the chart container.
  useEffect(() => {
    if (!ref.current) return;
    const el = ref.current;
    const overlay = el.querySelector<HTMLDivElement>(".tl-brush");
    if (!overlay) return;
    const span = bounds.end_ms - bounds.start_ms || 1;
    const left = ((value.start_ms - bounds.start_ms) / span) * 100;
    const width = ((value.end_ms - value.start_ms) / span) * 100;
    overlay.style.left = `${Math.max(0, left)}%`;
    overlay.style.width = `${Math.max(0.5, width)}%`;
  }, [bounds.start_ms, bounds.end_ms, value.start_ms, value.end_ms]);

  const applyPreset = (ms: number) => {
    const end = bounds.end_ms;
    const start = Math.max(bounds.start_ms, end - ms);
    onChange({ start_ms: start, end_ms: end });
  };

  return (
    <div className="tl-root" ref={ref}>
      {!compact && (
        <div className="tl-controls">
          <ButtonGroup minimal small>
            {PRESETS.map((p) => (
              <Button key={p.label} text={p.label} onClick={() => applyPreset(p.ms)} />
            ))}
          </ButtonGroup>
          {showCalendar && (
            <DateRangeControl
              value={value}
              onChange={onChange}
              bounds={bounds}
              highlightDates={highlightDates}
            />
          )}
          {hover != null && <Tag minimal intent="primary">{new Date(hover).toLocaleString()}</Tag>}
        </div>
      )}
      <div
        className="tl-chart"
        onMouseMove={(e) => {
          const rect = e.currentTarget.getBoundingClientRect();
          const frac = (e.clientX - rect.left) / rect.width;
          setHover(bounds.start_ms + frac * (bounds.end_ms - bounds.start_ms));
        }}
        onMouseLeave={() => setHover(null)}
      >
        <UPlotChart
          data={data}
          series={[{ label: summary?.label ?? "—", stroke: "#5c7080" }]}
          height={height}
          window={bounds}
          onZoom={(a, b) => onChange({ start_ms: a, end_ms: b })}
        />
        <div className="tl-brush" />
      </div>
    </div>
  );
}
