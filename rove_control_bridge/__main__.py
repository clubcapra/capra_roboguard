"""CLI entry point for rove_control_bridge.

Usage (from the parent of rove_control_bridge/):

    python -m rove_control_bridge --config rove_control_bridge/config/default.yaml

    # Override individual values at the command line:
    python -m rove_control_bridge --config config/default.yaml --listen-port 5010

Run build_protos.py once before first use:

    python rove_control_bridge/build_protos.py
"""
from __future__ import annotations

import argparse
import logging
import os
import sys
from pathlib import Path

if __package__ in (None, ""):
    _here = os.path.dirname(os.path.abspath(__file__))
    _parent = os.path.dirname(_here)
    if _parent not in sys.path:
        sys.path.insert(0, _parent)
    __package__ = os.path.basename(_here)

from .bridge import start
from .config import BridgeConfig, FlipperNodeIds, FlippersConfig, ListenConfig, SensorApiConfig, TracksConfig
from .config import load as load_config


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    p = argparse.ArgumentParser(description="RoveControl → rove_sensor_api bridge")
    p.add_argument(
        "--config",
        type=Path,
        default=None,
        metavar="FILE",
        help="YAML config file (required unless all options are passed as flags)",
    )
    # Allow selective overrides of the most commonly changed values.
    p.add_argument("--listen-host", default=None, help="Bind address for RoveControl packets")
    p.add_argument("--listen-port", type=int, default=None, help="UDP port to listen on")
    p.add_argument("--sensor-api-host", default=None, help="rove_sensor_api host")
    p.add_argument("--sensor-api-http-port", type=int, default=None, help="rove_sensor_api HTTP port")
    p.add_argument(
        "--tracks-strategy",
        choices=["velocity", "torque"],
        default=None,
        help="Track conversion strategy",
    )
    p.add_argument("--max-velocity", type=float, default=None, help="Max track velocity (rev/s)")
    p.add_argument("--max-torque", type=float, default=None, help="Max track torque (Nm)")
    p.add_argument("--send-rate", type=float, default=None, help="Keepalive send rate (Hz)")
    p.add_argument("--discover-timeout", type=float, default=None, help="ODrive discovery timeout (s)")
    p.add_argument("-v", "--verbose", action="store_true", default=False, help="Verbose logging")
    return p.parse_args(argv)


def merge_args_into_config(args: argparse.Namespace, cfg: BridgeConfig) -> BridgeConfig:
    """Apply any CLI overrides on top of the loaded config."""
    if args.listen_host is not None:
        cfg.listen.host = args.listen_host
    if args.listen_port is not None:
        cfg.listen.port = args.listen_port
    if args.sensor_api_host is not None:
        cfg.sensor_api.host = args.sensor_api_host
    if args.sensor_api_http_port is not None:
        cfg.sensor_api.http_port = args.sensor_api_http_port
    if args.tracks_strategy is not None:
        cfg.tracks.strategy = args.tracks_strategy
    if args.max_velocity is not None:
        cfg.tracks.max_velocity = args.max_velocity
    if args.max_torque is not None:
        cfg.tracks.max_torque = args.max_torque
    if args.send_rate is not None:
        cfg.send_rate_hz = args.send_rate
    if args.discover_timeout is not None:
        cfg.discover_timeout_s = args.discover_timeout
    if args.verbose:
        cfg.verbose = True
    return cfg


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)

    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
    )

    if args.config is not None:
        try:
            cfg = load_config(args.config)
        except Exception as exc:
            logging.error("Cannot load config file %s: %s", args.config, exc)
            return 1
    else:
        logging.info("No --config file given, using built-in defaults")
        cfg = BridgeConfig()

    cfg = merge_args_into_config(args, cfg)

    logging.info(
        "Config: listen=%s:%d  sensor_api=%s:%d  tracks.strategy=%s",
        cfg.listen.host, cfg.listen.port,
        cfg.sensor_api.host, cfg.sensor_api.http_port,
        cfg.tracks.strategy,
    )

    try:
        start(cfg)
    except RuntimeError as exc:
        logging.error("%s", exc)
        return 1
    except Exception:
        logging.exception("Bridge crashed")
        return 1

    return 0


if __name__ == "__main__":
    sys.exit(main())
