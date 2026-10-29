"""G3.1 per-attempt audit evidence for HISTORICAL, OFFLINE research only.

This module has no ROS dependencies. It never emits a motion command and does
not turn sampled MPC-CBF proxy feasibility into robot safety certification.
"""
import json
import math
from pathlib import Path

SCHEMA = "n01.offline.g3_frame_evidence.v1"


def _finite_or_none(value):
    if value is None:
        return None
    if (isinstance(value, bool) or not isinstance(value, (int, float))
            or not math.isfinite(value)):
        raise ValueError("nonfinite/invalid evidence metric")
    return float(value)


def _points(points):
    if not isinstance(points, list) or len(points) > 30:
        raise ValueError("invalid evidence trajectory")
    result = []
    for xy in points:
        if (not isinstance(xy, (list, tuple)) or len(xy) != 2 or
                any(isinstance(v, bool) or not isinstance(v, (int, float))
                    or not math.isfinite(v) for v in xy)):
            raise ValueError("nonfinite/invalid evidence position")
        result.append([float(xy[0]), float(xy[1])])
    return result


def frame_row(result):
    """Retain only genuine new numerical attempts, never periodic snapshots."""
    if result.get("schema") != "n01.offline.g3_mpc_cbf.v1" or (
            result.get("solver_executed") is not True):
        return None
    stamp = result.get("recorded_scan_stamp_ns")
    seq = result.get("path_reference_seq")
    if (type(stamp) is not int or stamp <= 0 or
            type(seq) is not int or seq <= 0 or
            result.get("source") !=
            "REAL_HISTORICAL_BAG_DETECTED_TARGET_HYPOTHESES" or
            result.get("safety_authorization") != "NEVER" or
            result.get("actuator_output") != "DISABLED"):
        raise ValueError("G3 attempt lacks offline provenance or safety boundary")
    def count(key):
        value = result.get(key)
        if type(value) is not int or not 0 <= value <= 10_000_000:
            raise ValueError("invalid G3 audit count: " + key)
        return value
    posthoc = result.get("post_hoc_reference_comparison")
    if type(posthoc) is not bool:
        raise ValueError("missing post-hoc provenance")
    nominal = _points(result.get("nominal_xy_m"))
    candidate = _points(result.get("candidate_xy_m"))
    status = result.get("status")
    if not isinstance(status, str) or not status or len(status) > 120:
        raise ValueError("invalid G3 solver status")
    if not nominal:
        raise ValueError("executed attempt missing nominal trajectory")
    if status.startswith("FEASIBLE_") and not candidate:
        raise ValueError("feasible attempt missing candidate trajectory")
    if not status.startswith("FEASIBLE_") and candidate:
        raise ValueError("nonfeasible attempt has candidate trajectory")
    moving = count("moving_constraint_tracks")
    checks = count("cbf_constraints_checked")
    rejected = count("cbf_constraints_rejected")
    if rejected > checks:
        raise ValueError("rejected constraints exceed checks")
    return {
        "schema": SCHEMA,
        "recorded_scan_stamp_ns": stamp,
        "path_reference_seq": seq,
        "path_origin": ("POST_HOC_HISTORICAL_COMPARISON" if posthoc else
                        "REPLAY_ORDER_REFERENCE_NOT_REALTIME_CONTROL"),
        "post_hoc_reference_comparison": posthoc,
        "contemporaneous_path_at_recorded_scan": False,
        "status": status,
        "solver_executed": True,
        "nominal_xy_m": nominal,
        "candidate_xy_m": candidate,
        "trajectory_changed": bool(candidate and candidate != nominal),
        "minimum_nominal_h_m2": _finite_or_none(result.get("minimum_nominal_h_m2")),
        "minimum_candidate_h_m2": _finite_or_none(result.get("minimum_candidate_h_m2")),
        "cbf_constraints_checked": checks,
        "cbf_constraints_rejected": rejected,
        "moving_constraint_tracks": moving,
        "source": "REAL_BAG_UNVERIFIED_MOVING_HYPOTHESES",
        "robot_model": "PLANAR_HOLONOMIC_POINT_MASS_UNVERIFIED_FOR_HIVE2",
        "actuator_output": "DISABLED",
        "safety_authorization": "NEVER",
        "collision_safety_verified": False,
    }


def append_frame(file_handle, result):
    row = frame_row(result)
    if row is not None:
        file_handle.write(json.dumps(row, ensure_ascii=False, sort_keys=True,
                                     allow_nan=False) + "\n")
        file_handle.flush()
    return row


def summarize(path):
    """Read a completed JSONL journal; reject duplicates/truncation/empty data."""
    path = Path(path)
    seen = set()
    counters = {"attempts": 0, "post_hoc_attempts": 0,
                "replay_order_attempts": 0, "feasible_nominal_only": 0,
                "feasible_dynamic_candidate_unverified": 0,
                "infeasible_attempts": 0, "cbf_constraints_checked": 0,
                "cbf_constraints_rejected": 0, "changed_trajectories": 0}
    with path.open(encoding="utf-8") as handle:
        for line in handle:
            if not line.endswith("\n"):
                raise ValueError("truncated G3 JSONL record")
            item = json.loads(line)
            if item.get("schema") != SCHEMA or item.get("solver_executed") is not True:
                raise ValueError("invalid frame evidence schema")
            key = (item["recorded_scan_stamp_ns"], item["path_reference_seq"],
                   item["post_hoc_reference_comparison"])
            if key in seen:
                raise ValueError("duplicate G3 attempt record")
            seen.add(key)
            if item.get("safety_authorization") != "NEVER" or (
                    item.get("actuator_output") != "DISABLED"):
                raise ValueError("unsafe G3 journal record")
            counters["attempts"] += 1
            counters["post_hoc_attempts" if item["post_hoc_reference_comparison"]
                     else "replay_order_attempts"] += 1
            status = item["status"]
            if status == "FEASIBLE_NOMINAL_ONLY_NO_DYNAMIC_CONSTRAINTS":
                counters["feasible_nominal_only"] += 1
            elif status == "FEASIBLE_CBF_CONSTRAINED_CANDIDATE_UNVERIFIED":
                counters["feasible_dynamic_candidate_unverified"] += 1
            elif status.startswith("INFEASIBLE_"):
                counters["infeasible_attempts"] += 1
            counters["cbf_constraints_checked"] += item["cbf_constraints_checked"]
            counters["cbf_constraints_rejected"] += item["cbf_constraints_rejected"]
            counters["changed_trajectories"] += bool(item["trajectory_changed"])
    if not counters["attempts"]:
        raise ValueError("no actual G3 solver attempts recorded")
    return {"schema": "n01.offline.g3_audit_summary.v1",
            "result": "OFFLINE_G3_FRAME_EVIDENCE_PRESENT_NOT_SAFETY_CERTIFIED",
            "recorded_evidence": str(path.name),
            "counts": counters,
            "safety_authorization": "NEVER", "actuator_output": "DISABLED"}


def main(argv=None):
    """Verify a finished replay's journal against observer's attempt floor."""
    import sys
    argv = list(sys.argv[1:] if argv is None else argv)
    if len(argv) != 3:
        raise SystemExit("usage: python3 -m n01_offline_integration.g3_frame_evidence JOURNAL SUMMARY REPORT")
    journal, summary_path, report_path = map(Path, argv)
    summary = summarize(journal)
    report = json.loads(report_path.read_text(encoding="utf-8"))
    expected = report.get("counts", {}).get("g3_solver_attempts")
    if type(expected) is not int or expected < 1:
        raise ValueError("observer has no genuine G3 solver attempts")
    if summary["counts"]["attempts"] < expected:
        raise ValueError("G3 journal has fewer attempts than observer")
    summary["observer_reported_solver_attempts"] = expected
    summary["journal_attempts_at_least_observer"] = True
    # An independent report counter may miss DDS messages; do not falsely
    # claim exact equality when counts differ.
    summary["counts_equal_observer"] = (summary["counts"]["attempts"] == expected)
    with summary_path.open("x", encoding="utf-8") as output:
        json.dump(summary, output, ensure_ascii=False, indent=2, allow_nan=False)
        output.write("\n")
    print(json.dumps(summary, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
