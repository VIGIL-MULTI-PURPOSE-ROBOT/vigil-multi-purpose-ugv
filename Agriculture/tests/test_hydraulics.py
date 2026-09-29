import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src/agri_ugv/scripts'))
import numpy as np
from hydraulics import cylinder

def test_virtual_work_and_jacobian():
    fixed=[-.1,0,.25];arm=[.45,0,.05]
    for q in np.linspace(-.22,.22,11):
        a=cylinder(q,600,fixed,arm);eps=1e-6
        derivative=(cylinder(q+eps,0,fixed,arm)['length_m']-cylinder(q-eps,0,fixed,arm)['length_m'])/(2*eps)
        assert abs(a['jacobian_m_per_rad']-derivative)<1e-7
        assert abs(a['force_n']*a['jacobian_m_per_rad']-600)<1e-7
        assert 0<a['stroke_m']<.3

def test_supplied_cad_standing_and_travel():
    import math
    from hydraulics import cylinder_from_notes
    standing=cylinder_from_notes(math.radians(-3.6),1000)
    assert abs(standing['length_m']-.5461)<.0004
    ends=[cylinder_from_notes(math.radians(q),0)['length_m'] for q in [-15.6,8.4]]
    assert abs((max(ends)-min(ends))-.1316)<.0005
    q=-.05;eps=1e-6
    derivative=(cylinder_from_notes(q+eps,0)['length_m']-cylinder_from_notes(q-eps,0)['length_m'])/(2*eps)
    assert abs(derivative-cylinder_from_notes(q,0)['jacobian_m_per_rad'])<1e-7
