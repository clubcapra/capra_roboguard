import { defineConfig } from "vite";
import react from "@vitejs/plugin-react";

const API_PORT = process.env.LOG_VIEWER_API_PORT ?? "8765";

export default defineConfig({
  plugins: [react()],
  server: {
    port: 5173,
    proxy: {
      "/api": `http://localhost:${API_PORT}`,
      "/ws": { target: `ws://localhost:${API_PORT}`, ws: true },
    },
  },
});
