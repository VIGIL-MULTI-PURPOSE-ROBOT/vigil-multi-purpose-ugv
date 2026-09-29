#!/usr/bin/python3
"""Open-chain cylinder length and virtual-work force conversion (SI units)."""
import math
import numpy as np

def cylinder(angle,torque,fixed,arm,retracted=.40):
    c,s=math.cos(angle),math.sin(angle);x,y,z=arm
    moving=np.array([c*x+s*z,y,-s*x+c*z]);derivative=np.array([-s*x+c*z,0.,-c*x-s*z])
    delta=moving-np.asarray(fixed);length=float(np.linalg.norm(delta))
    jacobian=float(delta@derivative/max(length,1e-9))
    if abs(jacobian)<.01:raise ValueError('Cylinder is near a toggle singularity')
    return {'length_m':length,'stroke_m':length-retracted,'jacobian_m_per_rad':jacobian,'force_n':torque/jacobian}


def cylinder_from_notes(angle,torque):
    """Signed extension force: dL/dtheta is negative for the supplied convention."""
    a,b=.33870,.52735
    included=math.radians(71.041)-angle
    length=math.sqrt(a*a+b*b-2*a*b*math.cos(included))
    jacobian=-a*b*math.sin(included)/length
    if abs(jacobian)<.01:raise ValueError('Cylinder near toggle singularity')
    return {'length_m':length,'stroke_m':length-.4782,'jacobian_m_per_rad':jacobian,'force_n':torque/jacobian}
