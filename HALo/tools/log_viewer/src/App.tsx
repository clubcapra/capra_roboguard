import { useEffect, useMemo, useState } from "react";
import {
  Alignment,
  Button,
  Card,
  Navbar,
  NonIdealState,
  Tag,
} from "@blueprintjs/core";
import { fetchSensors } from "./api";
import { useStore } from "./state";
import { SensorCard } from "./components/SensorCard";
import { AddSensorDialog } from "./components/AddSensorDialog";
import { Timeline } from "./components/Timeline";
import { DateRangeControl } from "./components/DateRangeControl";

export function App() {
  const sensors = useStore((s) => s.sensors);
  const setSensors = useStore((s) => s.setSensors);
  const mode = useStore((s) => s.mode);
  const setMode = useStore((s) => s.setMode);
  const sharedWindow = useStore((s) => s.sharedWindow);
  const setSharedWindow = useStore((s) => s.setSharedWindow);
  const cards = useStore((s) => s.cards);

  const [addOpen, setAddOpen] = useState(false);
  const [loadErr, setLoadErr] = useState<string | null>(null);

  useEffect(() => {
    fetchSensors()
      .then(setSensors)
      .catch((e) => setLoadErr(String(e)));
  }, []);

  const sensorById = useMemo(() => new Map(sensors.map((s) => [s.id, s])), [sensors]);

  // Bounds for the shared timeline: span the earliest log date we know about → now.
  const allDates = useMemo(() => {
    const set = new Set<string>();
    for (const s of sensors) for (const d of s.dates) set.add(d);
    return Array.from(set).sort();
  }, [sensors]);

  const bounds = useMemo(() => {
    const earliest = allDates
      .map((d) => Date.parse(`${d}T00:00:00Z`))
      .filter(Number.isFinite)
      .sort((a, b) => a - b)[0];
    return {
      start_ms: earliest ?? Date.now() - 24 * 3_600_000,
      end_ms: Date.now(),
    };
  }, [allDates]);

  const anyCardOnShared = cards.some((c) => c.useSharedWindow);

  return (
    <div className="app">
      <Navbar>
        <Navbar.Group align={Alignment.LEFT}>
          <Navbar.Heading>Rove log viewer</Navbar.Heading>
          <Navbar.Divider />
          <Button
            minimal
            active={mode === "historical"}
            icon="time"
            text="Historical"
            onClick={() => setMode("historical")}
          />
          <Button
            minimal
            active={mode === "live"}
            icon="record"
            text="Live"
            onClick={() => setMode("live")}
          />
        </Navbar.Group>
        <Navbar.Group align={Alignment.RIGHT}>
          {mode === "historical" && (
            <>
              <DateRangeControl
                value={sharedWindow}
                onChange={setSharedWindow}
                bounds={bounds}
                highlightDates={allDates}
              />
              <Navbar.Divider />
            </>
          )}
          <Tag minimal>{sensors.length} sensors</Tag>
          <Navbar.Divider />
          <Button intent="primary" icon="add" text="Add card" onClick={() => setAddOpen(true)} />
        </Navbar.Group>
      </Navbar>

      <div className="app-body">
        {loadErr && (
          <Card>
            <NonIdealState
              icon="error"
              title="Failed to load sensors"
              description={loadErr}
            />
          </Card>
        )}

        {!loadErr && cards.length === 0 && (
          <NonIdealState
            icon="series-add"
            title="No cards yet"
            description="Add a sensor card to start plotting."
            action={<Button intent="primary" icon="add" text="Add card" onClick={() => setAddOpen(true)} />}
          />
        )}

        <div className="card-grid">
          {cards.map((c) => {
            const s = sensorById.get(c.sensor);
            if (!s) {
              return (
                <Card key={c.uid}>
                  <NonIdealState
                    icon="warning-sign"
                    title={`Unknown sensor: ${c.sensor}`}
                    description="It may not have any logs yet."
                  />
                </Card>
              );
            }
            return <SensorCard key={c.uid} card={c} sensor={s} />;
          })}
        </div>
      </div>

      {mode === "historical" && anyCardOnShared && (
        <div className="shared-timeline">
          <Timeline
            bounds={bounds}
            value={sharedWindow}
            onChange={setSharedWindow}
            highlightDates={allDates}
            showCalendar={false}
          />
        </div>
      )}

      <AddSensorDialog isOpen={addOpen} onClose={() => setAddOpen(false)} />
    </div>
  );
}
