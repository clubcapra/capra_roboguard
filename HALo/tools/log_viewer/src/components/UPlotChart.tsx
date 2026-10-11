import { useEffect, useRef } from "react";
import uPlot from "uplot";
import "uplot/dist/uPlot.min.css";

export type Series = {
  label: string;
  /** CSS color. */
  stroke?: string;
};

export type UPlotChartProps = {
  /** [xs_ms[], ys[][]] — x in epoch milliseconds (uPlot expects seconds, we divide in the effect). */
  data: number[][];
  series: Series[];
  height?: number;
  /** When set, the chart will set its x-range to this window instead of fitting data. */
  window?: { start_ms: number; end_ms: number };
  /** Optional cursor highlight, in ms. */
  cursor_ms?: number | null;
  onZoom?: (start_ms: number, end_ms: number) => void;
};

const PALETTE = ["#6db8ff", "#ff8a6d", "#7be88a", "#f5d36b", "#c98aff", "#6dfff0", "#ff6db8"];

/** Tiny React wrapper around uPlot. We intentionally re-create the chart on series changes,
 *  but update data in place — recreating on every render is what would kill perf. */
export function UPlotChart(props: UPlotChartProps) {
  const containerRef = useRef<HTMLDivElement>(null);
  const plotRef = useRef<uPlot | null>(null);
  const seriesKey = props.series.map((s) => s.label).join("|");

  useEffect(() => {
    if (!containerRef.current) return;
    const el = containerRef.current;

    const opts: uPlot.Options = {
      width: el.clientWidth,
      height: props.height ?? 220,
      pxAlign: false,
      ms: 1, // x is milliseconds, not seconds
      scales: {
        x: { time: true },
      },
      legend: { show: true },
      cursor: {
        drag: { x: true, y: false, setScale: false },
      },
      hooks: {
        setSelect: [
          (u) => {
            if (!props.onZoom) return;
            if (u.select.width < 8) return;
            const a = u.posToVal(u.select.left, "x");
            const b = u.posToVal(u.select.left + u.select.width, "x");
            props.onZoom(Math.min(a, b), Math.max(a, b));
            u.setSelect({ left: 0, top: 0, width: 0, height: 0 }, false);
          },
        ],
      },
      series: [
        { label: "time" },
        ...props.series.map((s, i) => ({
          label: s.label,
          stroke: s.stroke ?? PALETTE[i % PALETTE.length],
          width: 1.25,
          spanGaps: true,
        })),
      ],
    };

    const u = new uPlot(opts, props.data as uPlot.AlignedData, el);
    plotRef.current = u;

    const ro = new ResizeObserver(() => u.setSize({ width: el.clientWidth, height: props.height ?? 220 }));
    ro.observe(el);

    return () => {
      ro.disconnect();
      u.destroy();
      plotRef.current = null;
    };
    // Recreate when the *shape* (series labels) changes; data and window updates use setData/setScale below.
  }, [seriesKey, props.height]);

  useEffect(() => {
    plotRef.current?.setData(props.data as uPlot.AlignedData);
  }, [props.data]);

  useEffect(() => {
    if (!plotRef.current || !props.window) return;
    plotRef.current.setScale("x", { min: props.window.start_ms, max: props.window.end_ms });
  }, [props.window?.start_ms, props.window?.end_ms]);

  return <div ref={containerRef} style={{ width: "100%" }} />;
}
