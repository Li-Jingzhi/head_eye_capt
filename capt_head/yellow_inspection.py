"""Read-only native-pixel sphere inspection using the new signed stereo geometry."""
import cv2
import numpy as np
from subpixel_ball import candidates

DETECTOR=dict(image_size=[1280,992],hsv_lower=[5,100,100],hsv_upper=[40,255,255],
              candidate_area_px=[200,8000],tracking_radius_px=72,max_center_step_px=45,
              min_edge_count=32,min_edge_coverage_deg=270,max_ellipse_rms_px=.5,max_ellipse_axis_ratio=1.35)


def triangulate(uv, model):
    P=[np.asarray(model[f'{eye}_P'],float) for eye in ('left','right')]
    h=cv2.triangulatePoints(P[0],P[1],np.asarray(uv[:2]).reshape(2,1),np.asarray(uv[2:]).reshape(2,1)).ravel()
    if abs(h[3])<1e-12:raise ValueError('point at infinity')
    xyz=h[:3]/h[3]
    if xyz[2]<=0:raise ValueError('point behind camera')
    errors=[]
    for i,p in enumerate(P):
        v=p@np.r_[xyz,1.];errors.append(float(np.linalg.norm(v[:2]/v[2]-uv[2*i:2*i+2])))
    return xyz,max(errors)


def sphere_center(uv,model,diameter_m=.04):
    original=np.asarray(uv,float);corrected=original.copy();xyz,_=triangulate(corrected,model)
    # Same silhouette-centre correction as the original project; diameter is an explicit assumption.
    for _ in range(6):
        coords=[]
        for i,eye in enumerate(('left','right')):
            K=np.asarray(model[f'{eye}_K']);ray=np.linalg.solve(K,np.r_[original[i*2:i*2+2],1.])
            factor=1-(diameter_m/2/xyz[2])**2
            pixel=K@np.array([ray[0]*factor,ray[1]*factor,1.]);coords.extend(pixel[:2]/pixel[2])
        corrected=np.asarray(coords);xyz,reprojection=triangulate(corrected,model)
    return xyz,corrected,reprojection


def inspect(frames,model):
    found=[candidates(image,DETECTOR) for image in frames]
    result=dict(candidate_counts=[len(x) for x in found],quality='REJECT',reason='no qualified sphere candidate',
                mean_luma=[float(cv2.cvtColor(f,cv2.COLOR_BGR2GRAY).mean()) for f in frames])
    if not all(found):return result
    pairs=[]
    sign=-np.sign(model['right_P'][0][3])
    for left in found[0]:
        for right in found[1]:
            uv=left['center']+right['center'];d=uv[0]-uv[2];dv=uv[1]-uv[3]
            if not 2<=sign*d<=576:continue
            pairs.append((abs(dv)+left['ellipse_rms_px']+right['ellipse_rms_px'],left,right,uv))
    if not pairs:
        result['reason']='no pair consistent with signed stereo baseline';return result
    _,left,right,uv=min(pairs,key=lambda x:x[0])
    xyz,pixels,error=sphere_center(uv,model)
    dv=pixels[1]-pixels[3]
    result.update(left=left,right=right,raw_stereo_pixels=uv,corrected_stereo_pixels=pixels.tolist(),
                  center_left_camera_m=xyz.tolist(),disparity_px=float(pixels[0]-pixels[2]),epipolar_dv_px=float(dv),
                  stereo_reprojection_px=error,sphere_diameter_assumed_m=.04,
                  quality='PASS' if abs(dv)<=.5 else 'REJECT',reason='ok' if abs(dv)<=.5 else 'epipolar error >0.5px')
    return result


def preview_detect(frames):
    """Per-eye2D detections for display only; never triangulate unpaired frames."""
    found=[candidates(image,DETECTOR) for image in frames]
    return dict(candidate_counts=[len(items) for items in found],
                left=found[0][0] if found[0] else None,
                right=found[1][0] if found[1] else None)
