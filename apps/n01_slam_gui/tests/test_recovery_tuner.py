"""Recovery tuning helpers: pure normalization and request building."""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from recovery_tuner import (GLOBAL_PARAM, LOCAL_PARAM, WAIT_PARAM, build_request,
                            describe, normalize)


def test_normalize_clamps_operator_input():
    assert normalize(0, 0, -5) == (1, 1, 0.0)
    assert normalize(999, 999, 9999) == (50, 20, 600.0)
    assert normalize(5, 3, 0) == (5, 3, 0.0)


def test_build_request_carries_three_typed_parameters():
    req = build_request(7, 2, 30)
    names = [p.name for p in req.parameters]
    assert names == [LOCAL_PARAM, GLOBAL_PARAM, WAIT_PARAM]
    assert req.parameters[0].value.integer_value == 7
    assert req.parameters[1].value.integer_value == 2
    assert req.parameters[2].value.double_value == 30.0


def test_describe_reports_stage_and_budget():
    text = describe({"recovery_stage": "global_replan",
                     "recovery_global_attempts": 2,
                     "recovery_global_limit": 3})
    assert "全局重规划" in text and "2/3" in text
    held = describe({"recovery_stage": "wait", "recovery_wait_s": 4.5,
                     "recovery_local_limit": 5, "recovery_global_limit": 3})
    assert "等待恢复" in held and "4.5" in held
    assert "未触发" in describe({})
