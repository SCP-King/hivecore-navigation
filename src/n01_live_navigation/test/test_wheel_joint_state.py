from types import SimpleNamespace as S
import pytest
from std_msgs.msg import Header
from n01_live_navigation.wheel_joint_state import convert,WHEELS

def sample():
 return S(header=Header(),joint_names=list(WHEELS),interface_values=[S(interface_names=['position','velocity'],values=[i+.5,-i*.1])for i in range(4)])

def test_real_wheel_values_preserved_without_leg_duplication():
 out=convert(sample())
 assert list(out.name)==list(WHEELS)
 assert list(out.position)==[.5,1.5,2.5,3.5]
 assert list(out.velocity)==pytest.approx([0,-.1,-.2,-.3])
 assert not out.effort

def test_missing_position_and_nonfinite_feedback_are_not_fabricated():
 m=sample();m.interface_values[0].interface_names=['velocity'];m.interface_values[0].values=[0]
 with pytest.raises(ValueError):convert(m)
 m=sample();m.interface_values[0].values[0]=float('nan')
 with pytest.raises(ValueError):convert(m)

def test_misaligned_interfaces_and_duplicates_are_rejected():
 m=sample();m.interface_values[0].values=[]
 with pytest.raises(ValueError):convert(m)
 m=sample();m.joint_names[1]=m.joint_names[0]
 with pytest.raises(ValueError):convert(m)
