"""Interpolate recorded physics states, without predicting future motion."""

import numpy as np


def quaternion_mix(a, b, fraction):
    a, b = np.asarray(a, float), np.asarray(b, float)
    a, b = a / np.linalg.norm(a), b / np.linalg.norm(b)
    dot = float(a @ b)
    if dot < 0:
        b, dot = -b, -dot
    if dot > 0.9995:
        q = a + fraction * (b - a)
        return (q / np.linalg.norm(q)).tolist()
    theta = np.arccos(np.clip(dot, -1, 1))
    return (
        (np.sin((1 - fraction) * theta) * a + np.sin(fraction * theta) * b)
        / np.sin(theta)
    ).tolist()


def interpolate_state(state, timestamp_ns):
    if not state:
        return None
    history = state.get("history", [])
    if not history:
        return state
    if timestamp_ns <= history[0]["timestamp_ns"]:
        return history[0]
    for a, b in zip(history, history[1:]):
        if timestamp_ns > b["timestamp_ns"]:
            continue
        span = b["timestamp_ns"] - a["timestamp_ns"]
        # Reset/respawn or a paused producer is a discontinuity, never a sweep.
        if (
            span <= 0
            or span > 250_000_000
            or np.linalg.norm(np.asarray(b["pos"]) - a["pos"]) > 1
        ):
            return b
        f = (timestamp_ns - a["timestamp_ns"]) / span
        result = {**b, "timestamp_ns": timestamp_ns}
        for key in ("pos", "q"):
            result[key] = (
                (1 - f) * np.asarray(a[key]) + f * np.asarray(b[key])
            ).tolist()
        result["orn"] = quaternion_mix(a["orn"], b["orn"], f)
        result["objects"] = {}
        for key, pose in b.get("objects", {}).items():
            previous = a.get("objects", {}).get(key, pose)
            result["objects"][key] = {
                "pos": (
                    (1 - f) * np.asarray(previous["pos"]) + f * np.asarray(pose["pos"])
                ).tolist(),
                "orn": quaternion_mix(previous["orn"], pose["orn"], f),
            }
        return result
    return history[-1]
