export type SensorInfo = {
  id: string;
  display_name: string;
  dates: string[];
  fields: string[];
  live: boolean;
};

export type RangePoint = {
  t_ms: number;
  values: (number | null)[];
};

export type RangeResult = {
  sensor: string;
  fields: string[];
  points: RangePoint[];
  downsampled: boolean;
};

export type Mode = "historical" | "live";

export type Card = {
  /** Local UI id — independent of sensor id so the same sensor can appear twice. */
  uid: string;
  sensor: string;
  fields: string[];
  /** If false, the card has its own local timeline strip and ignores the shared window. */
  useSharedWindow: boolean;
  /** Local window used when useSharedWindow=false. Epoch ms. */
  localWindow?: { start_ms: number; end_ms: number };
};

export type LivePoint = { t_ms: number; fields: Record<string, number | null> };
