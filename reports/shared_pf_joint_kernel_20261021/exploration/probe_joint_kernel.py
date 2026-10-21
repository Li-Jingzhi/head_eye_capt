import pickle,json,time
from pathlib import Path
import numpy as np
from scipy.spatial.distance import cdist
from scipy.linalg import cholesky,solve_triangular,cho_solve
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
from interactive_ball_calibration.compare_dual_pf import SIDES,fit_pair,subset
from interactive_ball_calibration.compare_pro_calibration import splits
from interactive_ball_calibration.pro_fit import metrics

def fit(data,kind,length,alpha,seed=None):
 if seed is None:seed=fit_pair(data,kind,'shared')
 block=12 if kind=='affine' else 6;start=2*block
 centers=np.concatenate([data[s]['points'] for s in SIDES]);K=np.exp(-cdist(centers,centers,'sqeuclidean')/(2*length**2));L=cholesky(K+alpha*np.eye(len(K)),lower=True)
 x0=np.r_[*(seed['models'][s]['parameters'][:block] for s in SIDES),seed['models']['right']['flange_to_ball_center_m']]
 lower=np.full(len(x0),-np.inf);upper=-lower;lower[start:]=-.2;upper[start:]=.2
 if kind=='affine':
  for i in range(2):lower[i*block:i*block+9]=x0[i*block:i*block+9]-.03;upper[i*block:i*block+9]=x0[i*block:i*block+9]+.03
 def maps(x):
  return {s:(x[i*block:i*block+9].reshape(3,3),x[i*block+9:i*block+12]) if kind=='affine' else (Rotation.from_rotvec(x[i*block:i*block+3]).as_matrix(),x[i*block+3:i*block+6]) for i,s in enumerate(SIDES)}
 def raw(x):
  mat=maps(x)
  return np.concatenate([(data[s]['poses'][:,:3,3]+np.einsum('nij,j->ni',data[s]['poses'][:,:3,:3],x[start:]))@mat[s][0].T+mat[s][1]-data[s]['points'] for s in SIDES])
 def fun(x):return (solve_triangular(L,raw(x),lower=True)/.001).ravel()
 def jac(x):
  mat=maps(x);J=np.zeros((len(K),3,len(x)));offset=0
  for i,s in enumerate(SIDES):
   Q=data[s]['poses'][:,:3,:3];b=data[s]['poses'][:,:3,3]+np.einsum('nij,j->ni',Q,x[start:]);n=len(b)
   for axis in range(3):J[offset:offset+n,axis,i*block+axis*3:i*block+axis*3+3]=b;J[offset:offset+n,axis,i*block+9+axis]=1
   J[offset:offset+n,:,start:]=np.einsum('ab,nbc->nac',mat[s][0],Q);offset+=n
  return (solve_triangular(L,J.reshape(len(K),-1),lower=True).reshape(len(K)*3,len(x))/.001)
 opt=least_squares(fun,x0,jac=jac if kind=='affine' else '2-point',bounds=(lower,upper),x_scale='jac',ftol=1e-9,xtol=1e-9,gtol=1e-9,max_nfev=150)
 if not opt.success:raise RuntimeError(opt.message)
 coeff=cho_solve((L,True),raw(opt.x));return dict(pf=opt.x[start:],maps=maps(opt.x),centers=centers,coeff=coeff,length=length,alpha=alpha)

def errors(data,m,s):
 p=data['points'];cor=p+np.exp(-cdist(p,m['centers'],'sqeuclidean')/(2*m['length']**2))@m['coeff'];M,t=m['maps'][s];ball=np.linalg.solve(M,(cor-t).T).T;q=data['poses']
 return ball-q[:,:3,3]-np.einsum('nij,j->ni',q[:,:3,:3],m['pf'])
if __name__=='__main__':
 data=pickle.loads(Path('/tmp/shared_kernel_data.pkl').read_bytes());outer={s:list(splits(np.arange(len(data[s]['points'])))) for s in SIDES}
 variants=[('affine',l,a) for l in [.03,.04,.06,.1] for a in [.3,1.,3.,10.]]+[('rigid_l2',l,a) for l in [.04,.1] for a in [1.,10.]]
 cv={str(v):{s:np.empty_like(data[s]['points']) for s in SIDES} for v in variants}
 for f in range(5):
  train={s:subset(data[s],outer[s][f][0]) for s in SIDES};seeds={k:fit_pair(train,k,'shared') for k in ['affine','rigid_l2']}
  for k,l,a in variants:
   m=fit(train,k,l,a,seeds[k])
   for s in SIDES:cv[str((k,l,a))][s][outer[s][f][1]]=errors(subset(data[s],outer[s][f][1]),m,s)
  print('fold',f,flush=True)
 scores={v:{s:metrics(cv[v][s]) for s in SIDES} for v in cv};Path('/tmp/joint_kernel_probe.json').write_text(json.dumps(scores,indent=2))
 for v in sorted(scores,key=lambda v:max(scores[v][s]['p95_mm'] for s in SIDES)):
  print(v,[(s,round(scores[v][s]['rms_mm'],4),round(scores[v][s]['p95_mm'],4),round(scores[v][s]['fraction_within_1mm'],4)) for s in SIDES],flush=True)
