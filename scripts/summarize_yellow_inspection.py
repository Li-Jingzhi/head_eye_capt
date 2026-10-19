#!/usr/bin/env python3
"""Summarize static native-pixel sphere repeatability, not absolute accuracy."""
import argparse
import collections
import json
from pathlib import Path
import numpy as np


def p95_delta(values):
    values=np.asarray(values)
    return float(np.percentile(abs(values-np.median(values)),95))


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('records');parser.add_argument('--model',required=True)
    args=parser.parse_args();path=Path(args.records)
    rows=[json.loads(s) for s in path.read_text().splitlines()]
    good=[r for r in rows if r['quality']=='PASS']
    if len(good)<50:raise ValueError('Fewer than50 qualified frames; no static stability conclusion')
    xyz=np.asarray([r['center_left_camera_m'] for r in good]);median=np.median(xyz,axis=0)
    disparity=np.asarray([r['disparity_px'] for r in good]);dv=np.asarray([r['epipolar_dv_px'] for r in good])
    model=json.loads(Path(args.model).read_text());f=model['left_K'][0][0];baseline=model['baseline_m']
    sensitivity=median[2]**2/(f*baseline)*1000
    fused=np.asarray([np.median(xyz[i:i+50],axis=0) for i in range(0,len(xyz)-49,50)])
    block_span=max((float(np.linalg.norm(a-b)*1000) for a in fused for b in fused),default=0.)
    report=dict(total_frames=len(rows),passed_frames=len(good),pass_fraction=len(good)/len(rows),
        rejection_reasons=dict(collections.Counter(r['reason'] for r in rows if r['quality']!='PASS')),
        median_xyz_left_camera_m=median.tolist(),median_signed_disparity_px=float(np.median(disparity)),
        disparity_abs_deviation_p95_px=p95_delta(disparity),
        xyz_radius_about_median_p95_mm=float(np.percentile(np.linalg.norm(xyz-median,axis=1),95)*1000),
        axis_abs_deviation_p95_mm=(np.percentile(abs(xyz-median),95,axis=0)*1000).tolist(),
        epipolar_absolute_p95_px=float(np.percentile(abs(dv),95)),epipolar_median_signed_px=float(np.median(dv)),
        ellipse_rms_p95_px={e:float(np.percentile([r[e]['ellipse_rms_px'] for r in good],95)) for e in ('left','right')},
        minimum_edge_coverage_deg={e:min(r[e]['edge_coverage_deg'] for r in good) for e in ('left','right')},
        pair_skew_all_p95_ms=float(np.percentile([r['pair_dequeue_skew_ms'] for r in rows],95)),
        pair_skew_pass_p95_ms=float(np.percentile([r['pair_dequeue_skew_ms'] for r in good],95)),
        depth_mm_per_disparity_pixel=float(sensitivity),disparity_pixels_corresponding_to_1mm=float(1/sensitivity),
        nonoverlapping_50_frame_blocks=len(fused),max_distance_between_block_medians_mm=block_span,
        mean_image_luma_p95_deviation={e:p95_delta([r['mean_luma'][i] for r in good]) for i,e in enumerate(('left','right'))},
        unique_source_pairs=len({tuple(r['source_pts_ns']) for r in good}),
        assumed_ball_diameter_m=.04,diameter_independently_measured=False,
        camera_stationary_joint_feedback_verified=False,verified_exposure_synchronization=False,
        conclusion='Single-point static repeatability passes1mm; absolute/workspace/FK accuracy NOT validated')
    assert report['unique_source_pairs']==len(good), 'Duplicate stereo observations'
    assert np.isfinite(xyz).all() and np.all(xyz[:,2]>0), 'Invalid reconstructed sphere positions'
    path.with_name('summary.json').write_text(json.dumps(report,indent=2))
    print(json.dumps(report,indent=2))


if __name__=='__main__':main()
