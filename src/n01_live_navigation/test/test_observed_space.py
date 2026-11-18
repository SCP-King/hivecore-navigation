import numpy as np
from n01_live_navigation.observed_space import path_observed


def test_centre_is_not_enough_to_authorize_unobserved_robot_footprint():
    polygon=np.array([[-.5,-.2],[.5,-.2],[.5,.2],[-.5,.2]])
    state=[[0.,0.,.7,0.]]
    assert not path_observed(state,polygon,np.array([[.05,.05,.7]]))
    x,y=np.meshgrid(np.arange(-1,1,.1)+.05,np.arange(-1,1,.1)+.05)
    points=np.column_stack((x.ravel(),y.ravel(),np.full(x.size,.7)))
    assert path_observed(state,polygon,points)
    assert path_observed([[0,0,.7,np.pi/2]],polygon,points)
    assert not path_observed([[.9,0,.7,0]],polygon,points)


def test_prefix_can_continue_before_unobserved_tail_with_distance_derived_speed():
    from n01_live_navigation.observed_space import executable_prefix
    import pytest
    states=[[x,0,0,0]for x in np.arange(0,1.02,.02)]
    valid=np.array([x[0]<.4 for x in states])
    prefix,speed=executable_prefix(states,valid,.8,.04,.6)
    assert len(prefix)>2 and prefix[-1][0]<.4 and 0<speed<.8
    length=prefix[-1][0]
    assert speed*.6+speed**2/(2*.04)+.06<=length+1e-9
    assert executable_prefix(states,np.ones(len(states),bool),.8,.04,.6)[1]==.8
    with pytest.raises(ValueError):executable_prefix(states,[False]*len(states),.8,.04,.6)
    with pytest.raises(ValueError):executable_prefix(states,[True,False]+[False]*(len(states)-2),.8,.04,.6)
