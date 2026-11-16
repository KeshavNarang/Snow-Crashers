import numpy as np, json, sys
from scipy.optimize import least_squares
W,H=3024,4032
def rot(a,b,c):  # yaw about z, pitch, roll -> world->cam
    # camera looks along heading yaw (deg CCW from +x), pitched down by b, rolled c
    y,p,r=np.radians([a,b,c])
    fwd=np.array([np.cos(y)*np.cos(p),np.sin(y)*np.cos(p),-np.sin(p)])
    right=np.array([np.sin(y),-np.cos(y),0.])
    down=np.cross(fwd,right)
    R=np.stack([right,down,fwd])
    cr,sr=np.cos(r),np.sin(r)
    return np.array([[cr,-sr,0],[sr,cr,0],[0,0,1]])@R
def proj(x,P):
    cam=x[:3]; R=rot(*x[3:6]); f=x[6]
    q=(np.atleast_2d(P)-cam)@R.T
    return np.column_stack([f*q[:,0]/q[:,2]+W/2, f*q[:,1]/q[:,2]+H/2]), q[:,2]
def solve(pts, lines, x0, fix_f=None, weights=None):
    """pts: [(world xyz, pixel uv, sigma_px)], lines: [(worldA, worldB, [observed pixels on it], sigma)]"""
    def res(x):
        if fix_f: x=np.r_[x[:6],fix_f]
        r=[]
        for Pw,uv,s in pts:
            u,_=proj(x,np.array(Pw)); r+=list((u[0]-uv)/s)
        for A,B,obs,s in lines:
            u,_=proj(x,np.array([A,B])); d=u[1]-u[0]; n=np.array([-d[1],d[0]])/np.linalg.norm(d)
            for o in obs: r.append(((np.array(o)-u[0])@n)/s)
        return np.array(r)
    x0=np.array(x0,float)
    if fix_f: x0=x0[:6]
    s=least_squares(res,x0)
    x=np.r_[s.x,fix_f] if fix_f else s.x
    J=s.jac; 
    try: cov=np.linalg.inv(J.T@J)
    except: cov=None
    return x, res(s.x), cov
def floor_point(x, uv, z=0.):
    cam=x[:3]; R=rot(*x[3:6]); f=x[6]
    d=R.T@np.array([(uv[0]-W/2)/f,(uv[1]-H/2)/f,1.])
    t=(z-cam[2])/d[2]; return cam+t*d
