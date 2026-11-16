import sys, numpy as np, cv2
sys.path.insert(0,str(__import__('pathlib').Path(__file__).resolve().parents[3] / 'goalnav'))
from localsim import messy_ladder as ml
from joint import run
from solve import proj
names=['side','diag1','diag2']; f=2850
x,L,sd=run(names,f,verbose=False)
files={'side':'Ladder_Side.jpg','diag1':'Ladder_Diag_One.jpg','diag2':'Ladder_Diag_Two.jpg'}
g=ml.ladder()
outs=[]
for i,n in enumerate(names):
    c=np.r_[x[6*i:6*i+6],f]; im=cv2.imread(str(__import__('pathlib').Path(__file__).resolve().parents[3] / 'photos' / files[n]))
    for name,(v,t) in g.items():
        col={'Ladder rails':(0,0,255),'Ladder metal':(255,0,255),'Ladder top':(0,255,0)}[name]
        uv,_=proj(c,v)
        for tri in t:
            pts=uv[tri].astype(int)
            cv2.polylines(im,[pts.reshape(-1,1,2)],True,col,2)
    # crop around ladder
    allv=np.vstack([v for v,_ in g.values()]); uv,_=proj(c,allv)
    x0,y0=np.maximum(uv.min(0).astype(int)-150,0); x1,y1=uv.max(0).astype(int)+150
    crop=im[y0:y1,x0:x1]; crop=cv2.resize(crop,(int(crop.shape[1]*1100/crop.shape[0]),1100))
    outs.append(crop)
cv2.imwrite('overlay.jpg',np.hstack(outs))
