import json,pickle
from pathlib import Path
import numpy as np
from scipy.spatial.distance import cdist
from scipy.linalg import cho_factor,cho_solve
from interactive_ball_calibration.compare_dual_pf import dataset,subset,fit_pair,SIDES
from interactive_ball_calibration.compare_pro_calibration import camera_to_arm,splits
from interactive_ball_calibration.pro_capture import ROOT
from interactive_ball_calibration.pro_fit import metrics
cache=Path('/tmp/shared_kernel_data.pkl')
if cache.exists():data=pickle.loads(cache.read_bytes())
else:
 data={s:dataset(ROOT/('datasets/drag_replay_20261020_091535_901340' if s=='right' else 'datasets/left_drag_replay_20261020_110645_673779'),s) for s in SIDES}
 cache.write_bytes(pickle.dumps(data))
lengths=[.04,.08,.16];alphas=[.1,1.,10.]
variants=[(seed,scope,l,a) for seed in ['rigid_l2','affine'] for scope in ['common','per_arm'] for l in lengths for a in alphas]
cv={str(v):{s:np.empty_like(data[s]['points']) for s in SIDES} for v in variants}
outer={s:list(splits(np.arange(len(data[s]['points'])))) for s in SIDES}
for f in range(5):
 train={s:subset(data[s],outer[s][f][0]) for s in SIDES}
 held={s:subset(data[s],outer[s][f][1]) for s in SIDES}
 for seed in ['rigid_l2','affine']:
  pair=fit_pair(train,seed,'shared'); targets={};maps={};base={}
  for s in SIDES:
   m=pair['models'][s];p=np.asarray(m['parameters']);M=p[:9].reshape(3,3) if seed=='affine' else __import__('scipy').spatial.transform.Rotation.from_rotvec(p[:3]).as_matrix()
   maps[s]=M
   q=train[s]['poses'];arm=q[:,:3,3]+np.einsum('nij,j->ni',q[:,:3,:3],m['flange_to_ball_center_m'])
   targets[s]=(arm-camera_to_arm(train[s]['points'],m))@M.T
   base[s]=camera_to_arm(held[s]['points'],m)
  for scope in ['common','per_arm']:
   for s in SIDES:
    src=np.concatenate([train[t]['points'] for t in SIDES]) if scope=='common' else train[s]['points']
    tar=np.concatenate([targets[t] for t in SIDES]) if scope=='common' else targets[s]
    D=cdist(src,src,'sqeuclidean');Dh=cdist(held[s]['points'],src,'sqeuclidean')
    for l in lengths:
     K=np.exp(-D/(2*l*l));Kh=np.exp(-Dh/(2*l*l))
     for a in alphas:
      c=cho_solve(cho_factor(K+a*np.eye(len(K))),tar)
      ball=base[s]+np.linalg.solve(maps[s],(Kh@c).T).T
      q=held[s]['poses'];e=ball-q[:,:3,3]-np.einsum('nij,j->ni',q[:,:3,:3],pair['models'][s]['flange_to_ball_center_m'])
      cv[str((seed,scope,l,a))][s][outer[s][f][1]]=e
 print('fold',f,flush=True)
scores={v:{s:metrics(cv[v][s]) for s in SIDES} for v in cv}
Path('/tmp/shared_kernel_probe.json').write_text(json.dumps(scores,indent=2))
for v in sorted(scores,key=lambda v:max(scores[v][s]['p95_mm'] for s in SIDES))[:20]:
 print(v,[(s,round(scores[v][s]['rms_mm'],4),round(scores[v][s]['p95_mm'],4),round(scores[v][s]['fraction_within_1mm'],4)) for s in SIDES],flush=True)
