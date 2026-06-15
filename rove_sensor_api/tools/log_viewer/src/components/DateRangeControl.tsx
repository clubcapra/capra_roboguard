import { useMemo } from "react";
import { Button, Popover } from "@blueprintjs/core";
import { DateRangePicker3 } from "@blueprintjs/datetime2";
import type { DateRange } from "@blueprintjs/datetime";

export type DateRangeControlProps = {
  value: { start_ms: number; end_ms: number };
  onChange: (next: { start_ms: number; end_ms: number }) => void;
  /** Outer bounds — used to disable days with no data. Optional. */
  bounds?: { start_ms: number; end_ms: number };
  /** Dates known to have logs (YYYY-MM-DD). Renders dots underneath them. */
  highlightDates?: string[];
};

/** Calendar-driven date+time range picker. Wraps Blueprint's DateRangePicker3 in a popover. */
export function DateRangeControl({ value, onChange, bounds, highlightDates }: DateRangeControlProps) {
  const range = useMemo<DateRange>(
    () => [new Date(value.start_ms), new Date(value.end_ms)],
    [value.start_ms, value.end_ms],
  );

  const highlightSet = useMemo(() => new Set(highlightDates ?? []), [highlightDates]);

  const minDate = bounds ? new Date(bounds.start_ms) : undefined;
  const maxDate = bounds ? new Date(bounds.end_ms) : undefined;

  const shortcuts = useMemo(() => {
    const now = bounds?.end_ms ?? Date.now();
    const mk = (label: string, span_ms: number) => ({
      label,
      dateRange: [new Date(now - span_ms), new Date(now)] as DateRange,
    });
    return [
      mk("Last hour", 3_600_000),
      mk("Last 6 hours", 6 * 3_600_000),
      mk("Today", new Date(now).getHours() * 3_600_000 + 60_000),
      mk("Last 24 hours", 24 * 3_600_000),
      mk("Last 7 days", 7 * 24 * 3_600_000),
      mk("Last 30 days", 30 * 24 * 3_600_000),
    ];
  }, [bounds?.end_ms]);

  const fmt = (d: Date) =>
    d.toLocaleString(undefined, {
      month: "short",
      day: "numeric",
      hour: "2-digit",
      minute: "2-digit",
    });

  return (
    <Popover
      placement="bottom-end"
      content={
        <DateRangePicker3
          value={range}
          onChange={(r) => {
            if (!r[0] || !r[1]) return;
            onChange({ start_ms: r[0].getTime(), end_ms: r[1].getTime() });
          }}
          minDate={minDate}
          maxDate={maxDate}
          shortcuts={shortcuts}
          timePrecision="minute"
          allowSingleDayRange
          singleMonthOnly={false}
          dayPickerProps={{
            modifiers: {
              hasData: (day: Date) => highlightSet.has(day.toISOString().slice(0, 10)),
            },
            modifiersClassNames: { hasData: "rdp-has-data" },
          }}
        />
      }
    >
      <Button icon="calendar" minimal rightIcon="caret-down">
        {fmt(new Date(value.start_ms))} → {fmt(new Date(value.end_ms))}
      </Button>
    </Popover>
  );
}
