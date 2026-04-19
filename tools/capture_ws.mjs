#!/usr/bin/env node

import fs from "node:fs";
import path from "node:path";
import process from "node:process";

function printUsage() {
  console.error("Usage: node tools/capture_ws.mjs ws://<esp32-ip>/ws/log [output.csv]");
}

function defaultOutputPath() {
  const stamp = new Date().toISOString().replace(/[:.]/g, "-");
  return path.resolve(process.cwd(), `esp32_capture_${stamp}.csv`);
}

function normalizeText(payload) {
  if (typeof payload === "string") {
    return payload;
  }
  if (payload instanceof ArrayBuffer) {
    return Buffer.from(payload).toString("utf8");
  }
  if (Buffer.isBuffer(payload)) {
    return payload.toString("utf8");
  }
  if (payload && typeof payload.text === "function") {
    return payload.text();
  }
  return String(payload ?? "");
}

const [, , urlArg, outputArg] = process.argv;

if (!urlArg) {
  printUsage();
  process.exit(1);
}

let wsUrl;
try {
  wsUrl = new URL(urlArg);
} catch (err) {
  console.error(`Invalid WebSocket URL: ${urlArg}`);
  process.exit(1);
}

if ((wsUrl.protocol !== "ws:") && (wsUrl.protocol !== "wss:")) {
  console.error(`Expected ws:// or wss:// URL, got ${wsUrl.protocol}`);
  process.exit(1);
}

if (typeof WebSocket !== "function") {
  console.error("This Node.js runtime does not expose a global WebSocket.");
  process.exit(1);
}

const outputPath = path.resolve(outputArg || defaultOutputPath());
const stream = fs.createWriteStream(outputPath, {
  flags: "a",
  encoding: "utf8",
});

let headerWritten = false;
let rowsWritten = 0;
let writeQueue = [];
let waitingDrain = false;
let closing = false;
let socketClosed = false;
const startedAt = Date.now();

function flushQueue() {
  while (!waitingDrain && writeQueue.length > 0) {
    const chunk = writeQueue.shift();
    if (!stream.write(chunk)) {
      waitingDrain = true;
    }
  }

  if (closing && !waitingDrain && writeQueue.length === 0 && !stream.closed) {
    stream.end();
  }
}

function enqueueWrite(text) {
  if (!text) {
    return;
  }
  writeQueue.push(text);
  flushQueue();
}

function handleCsvLine(line) {
  if (!line) {
    return;
  }
  if (line.startsWith("CSI_DATA_HEADER,")) {
    if (!headerWritten) {
      headerWritten = true;
      enqueueWrite(`${line}\n`);
    }
    return;
  }
  if (!line.startsWith("CSI_DATA,")) {
    return;
  }
  rowsWritten += 1;
  enqueueWrite(`${line}\n`);
}

function beginShutdown(reason) {
  if (closing) {
    return;
  }
  closing = true;
  console.error(reason);
  if (!socketClosed) {
    ws.close();
  }
  flushQueue();
}

stream.on("drain", () => {
  waitingDrain = false;
  flushQueue();
});

stream.on("error", (err) => {
  console.error(`File write failed: ${err.message}`);
  process.exitCode = 1;
  process.exit();
});

stream.on("close", () => {
  const elapsedSec = ((Date.now() - startedAt) / 1000).toFixed(1);
  console.error(`Saved ${rowsWritten} CSI rows to ${outputPath} in ${elapsedSec}s`);
  process.exit(process.exitCode ?? 0);
});

const ws = new WebSocket(wsUrl);

ws.addEventListener("open", () => {
  console.error(`Connected to ${wsUrl.href}`);
  console.error(`Writing CSV to ${outputPath}`);
});

ws.addEventListener("message", async (event) => {
  try {
    const text = await normalizeText(event.data);
    text.split("\n").forEach((line) => {
      const trimmed = line.trim();
      if (trimmed) {
        handleCsvLine(trimmed);
      }
    });
  } catch (err) {
    console.error(`Failed to process WebSocket payload: ${err.message}`);
  }
});

ws.addEventListener("error", () => {
  console.error("WebSocket error.");
});

ws.addEventListener("close", (event) => {
  socketClosed = true;
  beginShutdown(`WebSocket closed (${event.code}${event.reason ? `: ${event.reason}` : ""})`);
});

process.on("SIGINT", () => {
  beginShutdown("Stopping capture on Ctrl+C...");
});

process.on("SIGTERM", () => {
  beginShutdown("Stopping capture on SIGTERM...");
});
