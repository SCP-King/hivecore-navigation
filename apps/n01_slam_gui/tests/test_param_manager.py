"""Parameter registry: only our own tunables, documented and type-checked."""
import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import param_manager as pm


def test_registry_keys_unique_and_documented_in_chinese():
    names = [row[0] for row in pm.REGISTRY]
    assert len(names) == len(set(names))
    for name, _node, label, desc, _kind, _lo, _hi, _live in pm.REGISTRY:
        assert label and desc, name
        assert any('\u4e00' <= ch <= '\u9fff' for ch in label + desc), name


def test_registry_never_touches_client_owned_nodes():
    for _name, node, *_ in pm.REGISTRY:
        assert node in (pm.PLANNER, pm.MONITOR, pm.CONTROLLER)
        assert 'hivecore' not in node and 'chassis' not in node


def test_validate_rejects_out_of_range_and_bad_bool():
    assert pm.validate('recovery.local_attempts', 15) == 15
    with pytest.raises(ValueError):
        pm.validate('recovery.local_attempts', 0)
    with pytest.raises(ValueError):
        pm.validate('perception.scene_max_age_s', 99)
    assert pm.validate('recovery.auto_resume', 'true') is True
    assert pm.validate('recovery.auto_resume', '关') is False


def test_set_request_uses_declared_types():
    req = pm.build_set_request('recovery.local_attempts', 12)
    assert req.parameters[0].name == 'recovery.local_attempts'
    assert req.parameters[0].value.integer_value == 12
    req2 = pm.build_set_request('perception.scene_max_age_s', 1.2)
    assert req2.parameters[0].name == 'perception.scene_max_age_s'
    assert abs(req2.parameters[0].value.double_value - 1.2) < 1e-9
    req3 = pm.build_set_request('recovery.auto_resume', False)
    assert req3.parameters[0].value.bool_value is False


def test_options_use_parameter_name_as_value():
    opts = pm.options()
    assert 'recovery.local_attempts' in opts
    assert opts['recovery.local_attempts'].startswith('局部绕行次数 · ')
    assert all(key in pm.BY_KEY for key in opts)


def test_live_flags_describe_effect():
    live = {row[0]: row[7] for row in pm.REGISTRY}
    assert live['recovery.local_attempts'] is True
    assert live['perception.scene_max_age_s'] is True
    assert live['fsm.planning_horizon'] is False
    text = pm.describe('recovery.local_attempts', 5)
    assert '立即生效' in text and '当前值' in text
    assert '重启后生效' in pm.describe('fsm.planning_horizon', 10)
