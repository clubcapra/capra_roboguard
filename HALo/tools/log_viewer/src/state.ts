import { create } from "zustand";
import type { Card, Mode, SensorInfo } from "./types";

type State = {
  sensors: SensorInfo[];
  mode: Mode;
  /** Shared window for cards with useSharedWindow=true. Epoch ms. */
  sharedWindow: { start_ms: number; end_ms: number };
  cards: Card[];

  setSensors(s: SensorInfo[]): void;
  setMode(m: Mode): void;
  setSharedWindow(w: { start_ms: number; end_ms: number }): void;
  addCard(c: Card): void;
  removeCard(uid: string): void;
  updateCard(uid: string, patch: Partial<Card>): void;
};

const nowMs = () => Date.now();

export const useStore = create<State>((set) => ({
  sensors: [],
  mode: "historical",
  sharedWindow: { start_ms: nowMs() - 24 * 60 * 60 * 1000, end_ms: nowMs() },
  cards: [],

  setSensors: (sensors) => set({ sensors }),
  setMode: (mode) => set({ mode }),
  setSharedWindow: (sharedWindow) => set({ sharedWindow }),
  addCard: (card) => set((s) => ({ cards: [...s.cards, card] })),
  removeCard: (uid) => set((s) => ({ cards: s.cards.filter((c) => c.uid !== uid) })),
  updateCard: (uid, patch) =>
    set((s) => ({ cards: s.cards.map((c) => (c.uid === uid ? { ...c, ...patch } : c)) })),
}));

export function newUid(): string {
  return `c-${Math.random().toString(36).slice(2, 9)}`;
}
