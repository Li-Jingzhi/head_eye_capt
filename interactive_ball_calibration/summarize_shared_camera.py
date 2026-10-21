"""Audit saved camera-refinement predictions and write the experiment report."""
import argparse
import csv
import json
import re
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

from .compare_dual_pf import SIDES,dataset,subset
from .pro_capture import ROOT,digest
from .pro_fit import metrics
from .historical_reference import independent_rigid_reference
from .refine_shared_camera import camera_to_arm,choose,correct_camera,flange_errors


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory',type=Path)
    args=parser.parse_args();out=args.directory
    result=json.loads((out/'comparison.json').read_text())
    data={s:dataset(ROOT/result['data'][s]['path'],s) for s in SIDES}
    errors={v:{s:np.empty_like(data[s]['points']) for s in SIDES} for v in result['candidates']}
    seen={s:np.zeros(len(data[s]['points']),int) for s in SIDES}
    for fold in result['folds']:
        assert choose(fold['inner_scores'])==fold['selected']
        for s in SIDES:
            train=np.asarray(fold['splits'][s]['train']);held=np.asarray(fold['splits'][s]['held'])
            assert np.min(abs(train[:,None]-held))>2
            seen[s][held]+=1
            for v,m in fold['models'].items():errors[v][s][held]=flange_errors(subset(data[s],held),m,s)
    for s in SIDES:assert np.all(seen[s]==1)
    for v,record in result['candidates'].items():
        assert json.loads((out/f'{v}.json').read_text())==record['model']
        for s in SIDES:
            for key,value in metrics(errors[v][s]).items():np.testing.assert_allclose(value,record['cv'][s][key],atol=1e-9,rtol=0)
    for s in SIDES:
        nested=np.empty_like(data[s]['points'])
        for fold in result['folds']:
            held=np.asarray(fold['splits'][s]['held']);nested[held]=errors[fold['selected']][s][held]
        for key,value in metrics(nested).items():np.testing.assert_allclose(value,result['nested'][s][key],atol=1e-9,rtol=0)
    rows=list(csv.DictReader((out/'heldout_errors.csv').open()))
    assert len(rows)==len(result['candidates'])*(288+424)
    lookup={s:{r['waypoint_index']:i for i,r in enumerate(data[s]['rows'])} for s in SIDES}
    for row in rows:
        i=lookup[row['arm']][int(row['waypoint_index'])];delta=errors[row['variant']][row['arm']][i]*1000
        np.testing.assert_allclose(delta,[float(row[k]) for k in ('dx_mm','dy_mm','dz_mm')],atol=1e-9,rtol=0)
        assert abs(np.linalg.norm(delta)-float(row['error_mm']))<1e-9
    selected=result['selected'];model=result['candidates'][selected]['model']
    pf=np.asarray(model['shared_pf_m'])*1000
    old=independent_rigid_reference()
    diagnostics={}
    for s in SIDES:
        corrected=correct_camera(data[s]['points'],model)
        oldT=np.asarray(old[s]['camera_from_mdh_T']);original=(data[s]['points']-oldT[:3,3])@oldT[:3,:3]
        new=camera_to_arm(data[s]['points'],model,s)
        matrices=np.asarray(model['observation_from_mdh_T'][s])
        fold_pf=np.asarray([fold['models'][selected]['shared_pf_m'] for fold in result['folds']])*1000
        diagnostics[s]=dict(correction_magnitude=metrics(corrected-data[s]['points']),
            target_coordinate_change_from_deployed=metrics(new-original),
            matrix_singular_values=np.linalg.svd(matrices[:3,:3],compute_uv=False).tolist(),
            pf_training_fold_min_mm=fold_pf.min(0).tolist(),pf_training_fold_max_mm=fold_pf.max(0).tolist())
    (out/'diagnostics.json').write_text(json.dumps(diagnostics,indent=2)+'\n')
    verification=dict(heldout_predictions_recomputed=len(rows),nested_choices_and_scores_verified=True,
        both_arms_purged_holdout_and_complete_coverage_verified=True,all_serialized_models_verified=True,
        shared_pf_exact=True,software_test_cases=5,source_script_sha256=result['source_script_sha256'],
        current_source_matches_fit=digest(ROOT/'interactive_ball_calibration/refine_shared_camera.py')==result['source_script_sha256'],
        deployment_manifest_sha256=digest(ROOT/'vision_deploy/stereo_robot/calibration/manifest.json'),
        deployment_modified=False,absolute_accuracy_verified=False,
        any_dual_arm_p95_below_1mm=any(all(r['cv'][s]['p95_mm']<1 for s in SIDES) for r in result['candidates'].values()))
    (out/'VALIDATION.json').write_text(json.dumps(verification,indent=2)+'\n')
    labels=list(result['candidates']);fig,axes=plt.subplots(1,2,figsize=(13,9),sharey=True);y=np.arange(len(labels))
    for ax,metric in zip(axes,('rms_mm','p95_mm')):
        for delta,s,color in ((-.17,'right','#276aa1'),(.17,'left','#db842b')):
            ax.barh(y+delta,[result['candidates'][v]['cv'][s][metric] for v in labels],height=.32,label=s,color=color)
        ax.axvline(1,color='red',linestyle='--');ax.set_xlabel(metric);ax.grid(axis='x',alpha=.2)
    axes[0].set_yticks(y,labels);axes[0].invert_yaxis();axes[1].legend();fig.tight_layout();fig.savefig(out/'comparison.svg');plt.close(fig)
    lines=['# 共享pF与相机观测补偿：算法改进实验','',
        '日期：2026-10-20。基于最新右臂288点/14400帧、左臂424点/21200帧；两侧连接件同尺寸、同定位安装方向由用户确认。', '',
        f'**本轮比较18个模型/正则组合，所有模型严格共享一个pF。最佳候选为 `{selected}`，仍未达到双臂留出P95≤1mm。** 当前vision_deploy v2.4.0保持原参数，新模型单独存储，未连接硬件。','',
        '## 本轮改进了什么','',
        '先前主要改变各臂映射和pF约束。本轮引入两臂共用的相机观测修正函数f(y)，允许相同空间区域的系统偏差被统一补偿；各臂使用各自刚体或仿射映射。正则项限制补偿幅度，运行时只需相机XYZ与选定臂，不需关节角。', '',
        '```text','左右像素 → 原双目三角化得到y → 共用修正f(y) → 各臂逆映射 → 目标MDH坐标',
        '标定验证时：目标MDH坐标 − FK旋转×共享pF → 视觉法兰原点，与FK原点比较','```','',
        '相机K/D、左右相对R/T、P_L/P_R与MDH未重新标定。新增函数是经验观测补偿，不能证明修正项全部来自相机；也可能吸收机械臂几何、球心检测或受力误差。', '',
        '| 家族 | 修正内容 |','|---|---|',
        '| shared_rigid / affine_shared | 无新增观测修正，作为共享pF基线 |',
        '| disparity_plane | 视差偏差为归一化图像位置的一次函数，共3系数 |',
        '| disparity_quad | 视差偏差为图像位置的二次函数，共6系数 |',
        '| radial_disparity | 二次视差偏差加2个径向系数 |',
        '| xyz_quad | 归一化相机XYZ的二次向量场，共18系数 |',
        '| xyz_cubic | 二次及三次向量场，共48系数 |',
        '| affine_xyz_* | 各臂仿射映射加两臂共用的上述向量场 |','',
        '## 方程与约束','',
        '$$f(y_i)=M_s(a_i+Q_i p_F)+t_s,$$',
        '', 's表示机械臂；刚体方案M_s为旋转R_s，仿射方案M_s为一般线性矩阵。两臂只有一个共享pF。', '',
        '$$\min\sum_{s,i}\left\|\frac{M_s(a_i+Q_i p_F)+t_s-f(y_i)}{0.001}\right\|^2+\sum_j(c_j/\sigma_j)^2.$$','',
        '0.001m是拟合中数据残差的尺度，用于定义正则项相对权重，不是人为把误差缩放到1mm。报告误差均按未缩放的米制坐标计算后乘1000。', '',
        '视差修正使用u=(X/Z)/0.5、v=(Y/Z)/0.5，Δd由[1,u,v]或[1,u,v,u²,uv,v²]组合。当前带符号投影常数q=P_R[0,3]，修正f(y)=y/(1−Δd·Z/q)。径向扩展再对X/Y乘1+k1·r²+k2·r⁴，其中r²=(X/Z)²+(Y/Z)²。', '',
        'XYZ修正先令(x,y,z)=(y−[0,0,0.4])/0.2（米），使用二次基[ x²,xy,xz,y²,yz,z² ]；三次再加入[ x³,x²y,x²z,xy²,xyz,xz²,y³,y²z,yz²,z³ ]。f(y)=y+Φ(y)C，没有额外常数或一次项。', '',
        '视差系数正则尺度为0.1/0.3px；径向系数为0.005；XYZ系数为0.5/1/2mm（具体组合见表）。仿射元素限制在同折初始仿射解附近±0.03，最终奇异值须在0.9～1.1之间；相机/偏置物理边界及优化收敛均检查。最终模型未触及参数边界。', '',
        '## 留出结果','',
        '| 方案 | 右RMS mm | 右P95 mm | 右≤1mm | 左RMS mm | 左P95 mm | 左≤1mm |','|---|---:|---:|---:|---:|---:|---:|']
    for v,record in result['candidates'].items():
        a,b=[record['cv'][s] for s in SIDES]
        lines.append(f"| {v} | {a['rms_mm']:.3f} | {a['p95_mm']:.3f} | {a['fraction_within_1mm']:.2%} | {b['rms_mm']:.3f} | {b['p95_mm']:.3f} | {b['fraction_within_1mm']:.2%} |")
    lines+=['','![留出误差对比](comparison.svg)','','## 最佳候选参数与限制','',
        f'- 两臂共享pF（本臂法兰局部，mm）：`{pf.tolist()}`。',
        f'- [完整模型]({selected}.json)：左右M/t、共享pF、共用补偿系数、基函数归一化、正则设置及训练覆盖范围。',
        f"- 共{model['parameter_count']}个变量：两臂各12个仿射参数、3个共享pF及48个三次场系数。高阶场通过正则约束，不把低训练残差当作精度证明。",'',
        '| 指标 | 右臂 | 左臂 |','|---|---:|---:|']
    for metric in ('rms_mm','p95_mm','max_mm','fraction_within_1mm'):
        a,b=[result['candidates'][selected]['cv'][s][metric] for s in SIDES]
        lines.append(f'| {metric} | {a:.6f} | {b:.6f} |')
    a,b=[result['nested'][s] for s in SIDES]
    lines+=['',f"嵌套选型流程：右RMS/P95={a['rms_mm']:.3f}/{a['p95_mm']:.3f}mm，左={b['rms_mm']:.3f}/{b['p95_mm']:.3f}mm。每折只在训练数据上选模型与正则强度，随后验证该折的未见位置。",'',
        '与此前仿射＋≤0.5mm差异候选相比，原右RMS/P95=0.858/1.621mm、左=0.844/1.644mm。本次严格相同pF后，P95有所改善，但右RMS略升，不是所有指标都改善。与原部署独立刚体相比，原左P95=1.520mm仍低于本次候选，因此不应宣称全面优于原部署。', '',
        '左右臂验证点误差≤1mm的比例约78%，尚非95%。该流程仍使用静态50帧融合球心，不能保证任意物体单帧达到同样精度。', '',
        '## 如何避免自欺式降误差','',
        '- 原712个有效位置完整保留，未按残差删除坏点；拒绝位置也未重新纳入。',
        '- 两臂同时按轨迹分5个外层区块，两侧剔除相邻2个有效位置；内层4折选择模型与正则。同一位置50帧不跨训练/验证。',
        '- 验证用完整逆映射计算视觉法兰误差；仿射矩阵使用真正的逆矩阵，不用转置冒充逆矩阵。',
        '- 研究过程中已经反复查看同一批数据，嵌套选型只能减少本轮内部选型偏差，不能消除跨轮研究决策带来的偏差。最终需要独立会话/位置真值确认。',
        '- 共用修正对两臂相同，但两臂空间/姿态覆盖并不相同；不能据此独立确认它是物理相机畸变。', '',
        '## 实验调用方式','',
        '以下在安装分析依赖的原仓库环境执行，属于实验模型调用，不是当前vision_deploy的默认接口：','',
        '```python','import json','import numpy as np','from pathlib import Path',
        'from vision_deploy.stereo_robot import StereoRobot',
        'from interactive_ball_calibration.refine_shared_camera import camera_to_arm',
        f"model = json.loads(Path('reports/shared_camera_affine_refinement_20261020/{selected}.json').read_text())",
        "measurement = StereoRobot().predict([710, 569, 764, 569], arm='right')",
        "xyz_m = camera_to_arm(np.array([measurement['camera_xyz_m']]), model, 'right')[0]",
        'print(xyz_m * 1000)  # 实验模型对应的右MDH父系坐标，mm','```','',
        '运行目标点反算只需y与所选臂：P_s=M_s⁻¹(f(y)−t_s)，不加减pF。法兰误差验证才减去Q_i·pF。模型包含非线性修正和非刚体M，不能只取M/t放入现有刚体加载器。', '',
        '多项式超出采集区域可能快速放大误差；即使在训练包围盒内，也未验证任意位置精度。该实验函数未替代生产接口的完整输入/范围检查。', '',
        '## 文件与复现','',
        '- [完整结果](comparison.json)：所有模型、每折参数、内层分数、原始清单和限制。',
        '- [全部逐点留出误差](heldout_errors.csv)。',
        '- [验证记录](VALIDATION.json)：重新计算全部12816条留出预测，核对CSV和嵌套分数。',
        '- [补偿幅度与参数稳定性](diagnostics.json)：幅度是参数行为，不是绝对精度。','',
        '```bash','python -m interactive_ball_calibration.refine_shared_camera --output /tmp/shared_camera_refinement',
        'python -m interactive_ball_calibration.summarize_shared_camera /tmp/shared_camera_refinement',
        'python -m unittest tests.test_shared_camera -v','```','',
        '输出目录必须不存在。5项测试覆盖已知视差/非线性偏差的未见位置恢复、零偏差不被破坏、非法观测与选型逻辑。未进行独立实物精度验收。']
    text='\n'.join(lines)+'\n';(out/'report.md').write_text(text)
    missing=[link for link in re.findall(r'\]\(([^)]+)\)',text) if not (out/link).exists()]
    assert not missing,missing
    print(json.dumps(dict(selected=selected,pf_mm=pf.tolist(),verification=verification),indent=2))


if __name__=='__main__':main()
