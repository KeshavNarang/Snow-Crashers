import numpy as np, sys
from scipy.optimize import least_squares
from solve import rot, proj, W, H
N=3.048+0.024
def panel(x0=-3.07,x1=-2.12,y0=0.29,y1=0.36):
    return dict(TL=(x0,y0,1.75),WS=(x0,y0,0.94),WB=(x0,y0,0.28),ES=(x1,y1,0.94),EB=(x1,y1,0.28),drone=(-2.52,0.82,0.10))
photos={
 'side':dict(init=[-2.7,-3.9,1.5,84,13,0],anch={'WS':(905,1776),'WB':(910,2178),'EB':(1565,2145),'ES':(1565,1780),'drone':(1250,2235)},
             rail=[(700,1278),(1400,1276),(2300,1278)],feet={'A':(985,2590),'B':(1000,2688),'C':(1765,2590),'D':(1660,2458)}),
 'diag1':dict(init=[-3.5,-3.4,1.5,65,12,0],anch={'TL':(619,1245),'WS':(600,1900),'WB':(600,2382),'ES':(1320,1868),'EB':(1320,2269),'drone':(883,2393)},
             rail=[(500,1417),(1200,1419),(1900,1421)],feet={'A':(1010,2790),'B':(1290,2950),'C':(1890,2700),'D':(1610,2572)}),
 'diag2':dict(init=[-3.5,-3.3,1.5,64,12,0],anch={'TL':(574,1217),'WS':(556,1900),'WB':(554,2403),'ES':(1299,1880),'EB':(1299,2284),'drone':(854,2419)},
             rail=[(450,1404),(1150,1405),(1850,1407)],feet={'A':(950,2840),'B':(1220,3025),'C':(1870,2770),'D':(1590,2620)}),
}
SIG={'TL':10,'WS':10,'WB':12,'ES':15,'EB':15,'drone':25}
def ladder_feet(L):
    cx,cy,yaw,w,s=L
    a=np.radians(yaw); u=np.array([np.cos(a),np.sin(a)]); v=np.array([-u[1],u[0]])
    c=np.array([cx,cy])
    # A,B rear (west, -u), A north (+v); C,D front (east, +u), D north
    return {'A':c-u*s/2+v*w/2,'B':c-u*s/2-v*w/2,'C':c+u*s/2-v*w/2,'D':c+u*s/2+v*w/2}
def run(names, f, foot_sig=12, P=None, verbose=True, sideA_sig=None):
    P=P or panel()
    x0=[]
    for n in names: x0+=photos[n]['init']
    x0+=[-2.3,-0.55,17,0.50,0.97]
    def res(x):
        r=[]; L=x[-5:]; feet=ladder_feet(L)
        for i,n in enumerate(names):
            c=np.r_[x[6*i:6*i+6],f]; ph=photos[n]
            for k,uv in ph['anch'].items():
                u,_=proj(c,np.array(P[k])); r+=list((u[0]-uv)/SIG[k])
            A,B=np.array((-N,-N,1.524)),np.array((N,-N,1.524)); u,_=proj(c,np.array([A,B])); d=u[1]-u[0]; nn=np.array([-d[1],d[0]])/np.linalg.norm(d)
            for o in ph['rail']: r.append(((np.array(o)-u[0])@nn)/6)
            for k,uv in ph['feet'].items():
                s=foot_sig*(3 if (n=='side' and k=='A' and sideA_sig) else 1)
                u,_=proj(c,np.array([*feet[k],0.02])); r+=list((u[0]-uv)/s)
        r.append((L[3]-0.50)/0.03); r.append((L[4]-0.98)/0.04)
        return np.array(r)
    s=least_squares(res,np.array(x0,float))
    x=s.x; r=res(x)
    J=s.jac; cov=np.linalg.pinv(J.T@J)*max(1,np.mean(r**2))
    L=x[-5:]; sd=np.sqrt(np.diag(cov))[-5:]
    if verbose:
        print(names,f,'L=',np.round(L,3),'sd',np.round(sd,3),'rms',round(float(np.sqrt(np.mean(r**2))),2))
        for i,n in enumerate(names): print('   cam',n,np.round(x[6*i:6*i+6],2))
    return x,L,sd
if __name__=='__main__':
    for f in (2700,2850,3000):
        run(['side','diag1','diag2'],f)
    for names in (['side'],['diag1','diag2'],['side','diag1']):
        run(names,2850)
    print('-- side foot A down-weighted'); run(['side','diag1','diag2'],2850,sideA_sig=True)
    print('-- panel west end at -3.02'); run(['side','diag1','diag2'],2850,P=panel(x0=-3.02))
    print('-- panel east end at -2.08'); run(['side','diag1','diag2'],2850,P=panel(x1=-2.08))
    print('-- barrier 5cm north'); run(['side','diag1','diag2'],2850,P=panel(y0=0.34,y1=0.41))
