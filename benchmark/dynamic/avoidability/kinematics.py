"""Conservative level-ground enclosure counterfactual, never a controller input."""
from pathlib import Path
import math
import xml.etree.ElementTree as ET
import numpy as np
SDF=Path('/opt/ros/humble/share/nav2_bringup/worlds/waffle.model')
def transform(text):
    x,y,z,r,p,w=map(float,(text or '0 0 0 0 0 0').split())
    cr,sr,cp,sp,cw,sw=math.cos(r),math.sin(r),math.cos(p),math.sin(p),math.cos(w),math.sin(w)
    R=np.array([[cw*cp,cw*sp*sr-sw*cr,cw*sp*cr+sw*sr],[sw*cp,sw*sp*sr+cw*cr,sw*sp*cr-cw*sr],[-sp,cp*sr,cp*cr]])
    return np.array([x,y,z]),R
def geometry():
    parts=[]
    for link in ET.parse(SDF).findall('.//model/link'):
        lp,lr=transform(link.findtext('pose'))
        for col in link.findall('collision'):
            cp,cr=transform(col.findtext('pose')); pos=lp+lr@cp; R=lr@cr; g=col.find('geometry')
            if g.find('box') is not None:ext=np.abs(R)@ (np.array(list(map(float,g.findtext('box/size').split())))/2)
            elif g.find('sphere') is not None:ext=np.full(3,float(g.findtext('sphere/radius')))
            elif g.find('cylinder') is not None:
                axis=R[:,2];ext=np.abs(axis)*float(g.findtext('cylinder/length'))/2+float(g.findtext('cylinder/radius'))*np.sqrt(np.maximum(0,1-axis*axis))
            else:raise ValueError('unsupported collision')
            parts.append(dict(link=link.attrib['name'],collision=col.attrib['name'],lower=(pos-ext).tolist(),upper=(pos+ext).tolist()))
    return parts
def robot_bounds():
    parts=geometry();return [min(p['lower'][0] for p in parts),min(p['lower'][1] for p in parts),max(p['upper'][0] for p in parts),max(p['upper'][1] for p in parts)]
BOUNDS=robot_bounds()
def braking_distance(v,a,latency=0):return v*latency+v*v/(2*a)
def obstacle(t,start):return 1.6,np.clip(-1.1+(np.asarray(t)-start)*.25,-1.1,1.1)
def gaps(x,y,yaw,t,start,margin=0):
    """SAT separating-axis projection gap: lower bound on Euclidean gap, negative overlap depth proxy."""
    a,b,c,d=BOUNDS;a-=margin;b-=margin;c+=margin;d+=margin
    ct,st=np.cos(yaw),np.sin(yaw);lx=(a+c)/2;ly=(b+d)/2;hx=(c-a)/2;hy=(d-b)/2
    cx=x+ct*lx-st*ly;cy=y+st*lx+ct*ly;ox,oy=obstacle(t,start);dx=ox-cx;dy=oy-cy
    return np.maximum.reduce([np.abs(dx)-hx*np.abs(ct)-hy*np.abs(st)-.25,np.abs(dy)-hx*np.abs(st)-hy*np.abs(ct)-.25,np.abs(dx*ct+dy*st)-hx-.25*(np.abs(ct)+np.abs(st)),np.abs(-dx*st+dy*ct)-hy-.25*(np.abs(ct)+np.abs(st))])
def rollout(state,decision,start,policy='brake',decel=2.5,latency=0,margin=0,dt=.01,trace=False,wheel_accel=None):
    x,y,yaw,v,w=map(float,state);t=np.arange(decision,max(decision+.5,start+9.8)+dt/2,dt)
    age=np.maximum(0,t-decision-latency)
    vs=np.full(len(t),v) if policy=='hold' else np.sign(v)*np.maximum(0,abs(v)-decel*age)
    ws=np.full(len(t),w) if policy=='hold' else np.sign(w)*np.maximum(0,abs(w)-3.2*age)
    duration=abs(v)/decel
    if policy!='hold' and wheel_accel is not None:
        # Coordinated ramp to zero: both wheel accelerations and Nav2 command limits hold.
        duration=max(abs(v)/decel,abs(w)/3.2,abs(v-.287*w/2)/wheel_accel,abs(v+.287*w/2)/wheel_accel)
        scale=np.maximum(0,1-age/max(duration,1e-12));vs=v*scale;ws=w*scale
    yy=yaw+np.cumsum(np.r_[0,ws[:-1]*dt]);xx=x+np.cumsum(np.r_[0,vs[:-1]*np.cos(yy[:-1])*dt]);ys=y+np.cumsum(np.r_[0,vs[:-1]*np.sin(yy[:-1])*dt])
    gap=gaps(xx,ys,yy,t,start,margin);hit=np.flatnonzero(gap<=0)
    out=dict(safe=not len(hit),min_projection_gap_m=float(np.min(gap)),first_overlap_s=float(t[hit[0]]) if len(hit) else None,stop_x_m=float(xx[-1]),braking_distance_m=abs(v)*latency+abs(v)*duration/2)
    if trace:out['trace']={k:z.tolist() for k,z in dict(t=t,x=xx,y=ys,yaw=yy,v=vs,omega=ws,gap=gap).items()}
    return out