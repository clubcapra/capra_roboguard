import "@blueprintjs/core/lib/css/blueprint.css";
import "@blueprintjs/icons/lib/css/blueprint-icons.css";
import "@blueprintjs/select/lib/css/blueprint-select.css";
import "@blueprintjs/datetime2/lib/css/blueprint-datetime2.css";
import "react-day-picker/dist/style.css";
import "./styles.css";

import React from "react";
import { createRoot } from "react-dom/client";
import { FocusStyleManager } from "@blueprintjs/core";
import { App } from "./App";

FocusStyleManager.onlyShowFocusOnTabs();

const root = createRoot(document.getElementById("root")!);
root.render(
  <React.StrictMode>
    <App />
  </React.StrictMode>,
);
