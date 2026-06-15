import { useState } from "react";
import { Button, Dialog, DialogBody, DialogFooter, FormGroup, MenuItem } from "@blueprintjs/core";
import { Select, MultiSelect } from "@blueprintjs/select";
import { newUid, useStore } from "../state";
import type { SensorInfo } from "../types";

export function AddSensorDialog({ isOpen, onClose }: { isOpen: boolean; onClose: () => void }) {
  const sensors = useStore((s) => s.sensors);
  const addCard = useStore((s) => s.addCard);

  const [sensor, setSensor] = useState<SensorInfo | null>(null);
  const [fields, setFields] = useState<string[]>([]);

  const plottable = (sensor?.fields ?? []).filter((f) => f !== "timestamp_ns");

  const reset = () => {
    setSensor(null);
    setFields([]);
  };

  return (
    <Dialog title="Add sensor card" isOpen={isOpen} onClose={onClose} onClosed={reset}>
      <DialogBody>
        <FormGroup label="Sensor">
          <Select<SensorInfo>
            items={sensors}
            itemRenderer={(s, { handleClick, modifiers }) => (
              <MenuItem
                key={s.id}
                text={s.display_name}
                label={s.id}
                active={modifiers.active}
                onClick={handleClick}
              />
            )}
            itemPredicate={(q, s) =>
              s.id.toLowerCase().includes(q.toLowerCase()) ||
              s.display_name.toLowerCase().includes(q.toLowerCase())
            }
            onItemSelect={(s) => {
              setSensor(s);
              setFields(s.fields.filter((f) => f !== "timestamp_ns").slice(0, 3));
            }}
            popoverProps={{ minimal: true, matchTargetWidth: true }}
            fill
          >
            <Button
              alignText="left"
              fill
              text={sensor ? sensor.display_name : "Pick a sensor…"}
              rightIcon="caret-down"
            />
          </Select>
        </FormGroup>

        <FormGroup label="Fields">
          <MultiSelect<string>
            items={plottable}
            selectedItems={fields}
            tagRenderer={(f) => f}
            itemRenderer={(f, { handleClick, modifiers }) => (
              <MenuItem
                key={f}
                text={f}
                active={modifiers.active}
                icon={fields.includes(f) ? "tick" : "blank"}
                onClick={handleClick}
                shouldDismissPopover={false}
              />
            )}
            onItemSelect={(f) => setFields((cur) => (cur.includes(f) ? cur.filter((x) => x !== f) : [...cur, f]))}
            onRemove={(f) => setFields((cur) => cur.filter((x) => x !== f))}
            itemPredicate={(query, item) => item.toLowerCase().includes(query.toLowerCase())}
            placeholder={sensor ? "Select fields…" : "Pick a sensor first"}
            popoverProps={{ minimal: true }}
            resetOnSelect
          />
        </FormGroup>
      </DialogBody>
      <DialogFooter
        actions={
          <>
            <Button text="Cancel" onClick={onClose} />
            <Button
              intent="primary"
              text="Add"
              disabled={!sensor}
              onClick={() => {
                if (!sensor) return;
                addCard({
                  uid: newUid(),
                  sensor: sensor.id,
                  fields,
                  useSharedWindow: true,
                });
                onClose();
              }}
            />
          </>
        }
      />
    </Dialog>
  );
}
