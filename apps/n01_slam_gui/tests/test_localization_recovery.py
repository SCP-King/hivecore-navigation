from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from localization_recovery import RecoveryGate


def test_recovery_requires_time_and_advancing_measurements():
    gate=RecoveryGate()
    for i in range(4):assert gate.update(i*.3,True,i)=='waiting'
    assert gate.update(1.21,True,4)=='ready'
    gate=RecoveryGate()
    for i in range(299):assert gate.update(i*.1,True,1)=='waiting'
    assert gate.update(30.1,True,1)=='timeout'


def test_gap_resets_stable_interval():
    gate=RecoveryGate()
    for i in range(5):gate.update(i*.2,True,i)
    assert gate.update(.9,False,4)=='waiting'
    for i in range(4):assert gate.update(1.3+i*.3,True,5+i)=='waiting'
    assert gate.update(2.51,True,9)=='ready'


def test_short_unavailable_interval_is_excluded_without_discarding_all_progress():
    gate=RecoveryGate()
    for i in range(5):assert gate.update(i*.2,True,i)=='waiting'
    assert gate.update(.9,False,4)=='waiting'
    assert gate.update(1.0,True,5)=='waiting'
    assert gate.update(1.25,True,6)=='waiting'
    assert gate.update(1.31,True,7)=='ready'


def test_unavailable_chain_can_never_report_ready():
    gate=RecoveryGate()
    for i in range(15):assert gate.update(i*.1,i<12,i)=='waiting'
    assert gate.update(30.1,False,99)=='timeout'
