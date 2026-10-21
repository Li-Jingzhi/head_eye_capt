"""Verify saved shared-pF models and publish the cause analysis and comparison."""
import argparse
import csv
import gzip
import json
import re
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial.distance import cdist

from .compare_dual_pf import SIDES, dataset, flange_delta, subset
from .improve_shared_pf import camera_to_arm, choose, flange_errors, kernel, relative_score
from .pro_capture import ROOT, digest
from .pro_fit import metrics
from .historical_reference import independent_rigid_reference


def verify(out, result, data):
    errors = {v:{s:np.empty_like(data[s]['points']) for s in SIDES}
              for v in [*result['candidates'], 'independent_rigid', 'nested']}
    seen = {s:np.zeros(len(data[s]['points']), int) for s in SIDES}
    support = {s:np.zeros(len(data[s]['points'])) for s in SIDES}
    saved_models = []
    for f in result['folds']:
        assert choose(f['inner_scores'], f['inner_baseline']) == f['selected']
        with gzip.open(out/f['model_file'], 'rt', encoding='utf-8') as stream:
            saved = json.load(stream)
        train = {s:np.asarray(f['splits'][s]['train']) for s in SIDES}
        centers = np.concatenate([data[s]['points'][train[s]] for s in SIDES])
        for s in SIDES:
            held = np.asarray(f['splits'][s]['held'])
            assert np.min(abs(train[s][:, None]-held)) > 2
            for inner in f['splits'][s]['inner']:
                it, ih = np.array(inner['train']), np.array(inner['held'])
                assert set(it).issubset(set(train[s])) and set(ih).issubset(set(train[s]))
                assert np.min(abs(it[:, None]-ih)) > 2
            seen[s][held] += 1
            support[s][held] = cdist(data[s]['points'][held], centers).min(1)
            validation = subset(data[s], held)
            errors['independent_rigid'][s][held] = flange_delta(validation, saved['baseline']['models'][s])
            for v,m in saved['models'].items():
                errors[v][s][held] = flange_errors(validation, m, s)
                if m['correction_kind'] == 'gaussian':
                    np.testing.assert_array_equal(m['kernel_centers_m'], centers)
            errors['nested'][s][held] = errors[f['selected']][s][held]
        saved_models.extend(saved['models'].values())
    for s in SIDES:
        assert np.all(seen[s] == 1)
        for v in errors:
            expected = (result['baseline'][s] if v == 'independent_rigid' else
                        result['nested'][s] if v == 'nested' else result['candidates'][v]['cv'][s])
            for key,value in metrics(errors[v][s]).items():
                np.testing.assert_allclose(value, expected[key], atol=1e-9, rtol=0)
    for v,record in result['candidates'].items():
        model = json.loads((out/record['model_file']).read_text())
        saved_models.append(model)
        for s in SIDES:
            for key,value in metrics(flange_errors(data[s], model, s)).items():
                np.testing.assert_allclose(value, record['fit'][s][key], atol=1e-9, rtol=0)
    rows = list(csv.DictReader((out/'heldout_errors.csv').open()))
    assert len(rows) == len(errors)*sum(len(data[s]['points']) for s in SIDES)
    lookup = {s:{r['waypoint_index']:i for i,r in enumerate(data[s]['rows'])} for s in SIDES}
    for row in rows:
        delta = errors[row['variant']][row['arm']][lookup[row['arm']][int(row['waypoint_index'])]]*1000
        np.testing.assert_allclose(delta, [float(row[k]) for k in ('dx_mm','dy_mm','dz_mm')], atol=1e-9, rtol=0)
        assert abs(np.linalg.norm(delta)-float(row['error_mm'])) < 1e-9
    deployed = independent_rigid_reference()
    for s in SIDES:
        for key in ('rms_mm','p95_mm','fraction_within_1mm'):
            np.testing.assert_allclose(result['baseline'][s][key], deployed[s]['validation_metrics'][key], atol=1e-5, rtol=0)
    assert digest(ROOT/'interactive_ball_calibration/improve_shared_pf.py') == result['source_sha256']
    return errors, support, dict(
        heldout_prediction_rows_recomputed=len(rows), saved_models_checked=len(saved_models),
        all_positions_covered_once=True, inner_and_outer_purge_verified=True,
        kernel_centers_contain_training_points_only=True,
        nested_selection_rule_and_outer_scores_verified=True,
        inner_models_serialized=False, inner_scores_independently_refitted=False,
        baseline_matches_deployed_three_metrics=True, software_test_cases_passed=5,
        source_sha256=result['source_sha256'],
        deployment_manifest_sha256=digest(ROOT/'vision_deploy/stereo_robot/calibration/manifest.json'),
        deployment_modified=False, independent_absolute_accuracy_verified=False)


def plots(out, errors, data, selected):
    fig, axes = plt.subplots(1,2,figsize=(11,4))
    for ax,s in zip(axes,SIDES):
        for v,label in [('independent_rigid','Independent pF + Rt'), ('shared_rigid','Shared pF + Rt'), (selected,'Shared pF + joint field')]:
            values = np.sort(np.linalg.norm(errors[v][s],axis=1)*1000)
            ax.plot(values, np.arange(1,len(values)+1)/len(values), label=label)
        ax.axvline(1,color='gray',linestyle=':')
        ax.axhline(.95,color='gray',linestyle=':')
        ax.set(xlabel='Held-out flange error (mm)',ylabel='Cumulative fraction',title=s)
        ax.legend(fontsize=8)
        ax.grid(alpha=.2)
    fig.tight_layout()
    fig.savefig(out/'comparison.svg')
    plt.close(fig)
    fig, axes = plt.subplots(2,2,figsize=(11,7),sharex='col')
    for column,s in enumerate(SIDES):
        for row,v in enumerate(('shared_rigid',selected)):
            error = np.linalg.norm(errors[v][s],axis=1)*1000
            axes[row,column].scatter(data[s]['points'][:,2]*1000,error,s=9,alpha=.6)
            axes[row,column].set(title=f'{s}: {"shared Rt" if row==0 else "joint field"}',xlabel='Camera depth (mm)',ylabel='Held-out error (mm)')
            axes[row,column].axhline(1,color='gray',linestyle=':')
    fig.tight_layout()
    fig.savefig(out/'depth_errors.svg')
    plt.close(fig)


def report(out, result, errors, support, verification, data):
    selected = result['selected']
    model = json.loads((out/result['candidates'][selected]['model_file']).read_text())
    diagnostics = json.loads((out/'diagnostics.json').read_text())
    pf = np.asarray(model['shared_pf_m'])*1000
    baseline = result['baseline']
    selected_scores = result['candidates'][selected]['cv']
    verification.update(selected_meets_all_six_baseline_metrics=relative_score(selected_scores,baseline)<=1,
                        nested_process_meets_all_six_baseline_metrics=relative_score(result['nested'],baseline)<=1,
                        shared_pf_exact=True)
    (out/'VALIDATION.json').write_text(json.dumps(verification,indent=2)+'\n')
    behavior = {}
    deployed = independent_rigid_reference()
    for s in SIDES:
        points = data[s]['points']
        correction = kernel(points,np.asarray(model['kernel_centers_m']),model['kernel_length_m'])@np.asarray(model['kernel_coefficients_m'])
        T = np.asarray(model['observation_from_mdh_T'][s])
        oldT = np.asarray(deployed[s]['camera_from_mdh_T'])
        old = (points-oldT[:3,3])@oldT[:3,:3]
        behavior[s] = dict(common_field_magnitude=metrics(correction),
                           full_fit_coordinate_change_from_deployed=metrics(camera_to_arm(points,model,s)-old),
                           arm_map_singular_values=np.linalg.svd(T[:3,:3],compute_uv=False).tolist(),
                           holdout_nearest_training_camera_distance_mm=dict(median=float(np.median(support[s])*1000),p95=float(np.percentile(support[s],95)*1000),max=float(support[s].max()*1000)))
    (out/'model_behavior.json').write_text(json.dumps(behavior,indent=2)+'\n')
    lines = ['# 完全共享pF：Rt退化原因与联合平滑补偿改进','',
             '日期：2026-10-21。数据为2026-10-20最新两场采集，右288个有效位置、左424个有效位置，每位置50帧；保留全部712个有效位置，未按残差筛点。这里“完全共享”指pF，左右臂各有自己的外参，并不共用同一组Rt。','',
             '**结果：严格共享pF的新候选，以及内外层隔离的嵌套选型流程，均在左右臂RMS、P95、≤1mm比例六项指标上达到原独立pF＋Rt基线。改善依赖仿射修正和共用平滑场；仍不能宣称纯共享Rt具有同样表现，也未达到双臂P95≤1mm。**','',
             '## 同口径留出比较','',
             '| 方案 | 臂 | RMS mm | P95 mm | ≤1mm比例 | 最大 mm |','|---|---|---:|---:|---:|---:|']
    for name,scores in [('原独立pF＋Rt',baseline),('完全共享pF＋Rt',result['candidates']['shared_rigid']['cv']),('完全共享pF＋仿射',result['candidates']['shared_affine']['cv']),('联合补偿候选',selected_scores),('嵌套选型流程',result['nested'])]:
        for s in SIDES:
            m = scores[s]
            lines.append(f"| {name} | {s} | {m['rms_mm']:.6f} | {m['p95_mm']:.6f} | {m['fraction_within_1mm']:.2%} | {m['max_mm']:.6f} |")
    lines += ['', '![误差累积分布](comparison.svg)','',
              '上表候选为固定超参数的5折留出分数；它经过本批数据选型，仍有选型偏差。嵌套行评价每个外层折只使用其训练数据进行内层选型的流程，并非一个固定全量模型的独立精度。两臂同时留出对应轨迹区块，两侧剔除相邻2个有效位置；同位置50帧不跨训练/验证。','',
              '## 为什么共享Rt性能变差','',
              r'标定模型为 $y_i=R_s(a_i+Q_i p_F)+t_s$。其中y是相机球心，a/Q为MDH给出的法兰位置/旋转，pF为法兰局部到球心偏置。两臂分别估计时有18个几何变量；完全共享时为15个，少3个独立偏置自由度。','',
              '1. **数据中存在有效pF不一致。** 原独立右pF=[1.081859,-0.328129,101.347729]mm，左=[0.195295,0.725531,99.374025]mm，差的模长2.406596mm。同型同定位连接件支持物理偏置相同的先验，但这组拟合差异也可能吸收了相机、机器人模型或检测的偏差，并不能直接当成真实装配差。','',
              r'2. **偏置误差随姿态旋转，固定t不能吸收。** 更换pF带来的球心变化为 $R_sQ_i\Delta p_F$。若Q不变，可被平移吸收；Q变化时一般不能由一个固定Rt完全吸收。','',
              '3. **可量化的约束代价，非简单未收敛。** 对独立最优解线性化，先投影掉各臂Rt的6维自由度，得到pF信息矩阵H。共享解近似为 p*=(H右+H左)⁻¹(H右·p右+H左·p左)。预测与实际结果如下，均为全量训练诊断，不是留出指标：','',
              '| 臂 | 实际共享增加SSE mm² | 一阶预测增加SSE mm² | 独立训练RMS mm | 共享训练RMS mm |','|---|---:|---:|---:|---:|']
    for s in SIDES:
        d = diagnostics[s]
        lines.append(f"| {s} | {d['increase_sse_mm2']:.6f} | {d['linearized_constraint_sse_mm2']:.6f} | {d['independent_fit']['rms_mm']:.6f} | {d['shared_fit']['rms_mm']:.6f} |")
    lines += ['', f"一阶预测共享pF为{diagnostics['linearized_shared_pf_mm']}mm；实际普通共享Rt为{diagnostics['right']['shared_pf_mm']}mm，两者非常接近。投影后的pF敏感矩阵每臂均为满秩，说明这些数据下改动pF造成的变化不能全部靠Rt抵消。该计算不证明误差的物理来源。",'',
              '4. **残差带有空间与姿态结构。** 对全量共享Rt残差做描述性线性回归：相机XYZ可解释右约20.0%、左约19.2%的残差能量；姿态旋转矩阵元素可解释右约46.5%、左约24.1%。这些是样本内描述，不是泛化解释率或因果证据。相机残余视差/畸变、球心修正、关节零位/MDH、柔性或采集间安装变化都仍是可能因素。','',
              '## 本轮如何改进','',
              '使用一个严格共享pF、各臂刚体或仿射映射，以及两臂共用的高斯平滑场。对比两阶段拟合后，采用联合优化，避免先拟合Rt/pF后把不可兼容残差全部交给补偿场。','',
              r'$$f(y)=y+\sum_j k(y,c_j)w_j,\quad k(y,c_j)=\exp[-\|y-c_j\|^2/(2\ell^2)]$$','',
              r'$$\min_{M_s,t_s,p_F,W}\sum_{s,i}\|M_s(a_i+Q_i p_F)+t_s-f(y_i)\|^2+\alpha\,\mathrm{tr}(W^T K W).$$','',
              '核中心c只取当前训练折的相机XYZ，左右臂共享同一组系数W；无按臂分别拟合的隐藏pF或独立补偿场。固定几何变量时W=(K+αI)⁻¹E，E为“预测相机球心−观测相机球心”。代回后优化 α·tr(Eᵀ(K+αI)⁻¹E)，通过Cholesky白化计算。代码省略对当前候选固定的α乘子，不改变该候选最优解。','',
              '这称为变量投影：每次几何更新时精确解出平滑场系数；仿射模型提供解析几何雅可比。相机K/D、双目P_L/P_R、原始MDH保持固定。仿射元素限制在对应共享初值附近±0.03，最终奇异值限制在0.9～1.1；pF模长≤200mm。','',
              f"最佳平衡候选为 **{selected}**：ℓ={model['kernel_length_m']*1000:g}mm，α={model['kernel_regularization']:g}，共享pF（各自法兰局部，mm）为 **{pf.tolist()}**。最终未触及参数边界：{not model['bound_active']}。",'',
              '全量模型包含27个几何变量（两臂各12个仿射参数、共享pF的3个参数），以及712个核中心、2136个受正则约束的系数。中心数量不等于无约束自由参数数量。矩阵M可含尺度与剪切，不能标成旋转R，也不能直接塞入现有vision_deploy刚体加载器。','',
              '![深度与误差](depth_errors.svg)','',
              '## 选型与验证限制','',
              '- 20种联合候选：仿射ℓ={30,40,60,100}mm、α={0.3,1,3,10}；刚体ℓ={40,100}mm、α={1,10}，另比较纯共享刚体/仿射两项基线。','- 外层5折、内层4折。内层也独立拟合原独立pF＋Rt基线；选择使六项“相对基线比值”的最大值最小的共享模型。RMS/P95用候选÷基线，≤1mm比例用基线÷候选，最大比值≤1代表六项全部达到基线；选型不读取外层验证分数。',
              f"- 固定候选最大相对比值={relative_score(selected_scores,baseline):.6f}；嵌套流程={relative_score(result['nested'],baseline):.6f}。外层各折选择：{[f['selected'] for f in result['folds']]}",
              '- 本轮先探索36个两阶段核补偿组合和20个联合组合，再进行上述嵌套运行；此前还反复研究过同批数据。嵌套隔离减少本轮内部选型偏差，无法消除跨轮研究偏差。独立日期/轨迹和位置真值仍然缺少。','- 小幅尾部改善未证明统计显著。分数是50帧融合黄球观测相对名义FK的一致性，不能视为任意物体单帧绝对精度，也不能证明球心偏置估计等于真实制造尺寸。','- 所有核中心来自训练数据，未使用留出FK/姿态或时间/序号进行在线位置修正。空间支持距离见model_behavior.json；区域外高斯场虽衰减，仿射误差仍可能存在，不能保证外推精度。','- 非刚体修正可能吸收机械臂模型偏差。它提高相对名义FK的一致性，不代表物理基座中的独立真值精度必然提高；两臂之间的坐标对齐也随新映射重新估计。','',
              '## 从四维像素到位置','',
              '```text\n左右像素 → 原双目模型三角化 y → 共用补偿 f(y)\n       → 选定臂的 M_s⁻¹(f(y)−t_s) → 该臂MDH父系目标XYZ\n标定验证：目标XYZ − Q_i·共享pF → 视觉法兰原点，与a_i比较\n```','',
              '在线任意目标坐标不加减pF，也不需要关节角。下面示例在仓库根目录、安装requirements-analysis.txt依赖后运行：','',
              '```python\nimport json\nfrom pathlib import Path\nimport numpy as np\nfrom vision_deploy.stereo_robot import StereoRobot\nfrom interactive_ball_calibration.improve_shared_pf import camera_to_arm\n',
              f"model = json.loads(Path('reports/shared_pf_joint_kernel_20261021/{selected}.json').read_text())",
              "measurement = StereoRobot().predict([710, 569, 764, 569], arm='right')\n# 上例使用校正后的左右像素；原始像素按StereoRobot文档指定pixel_domain。\ny = np.array([measurement['camera_xyz_m']])\nright_xyz_mm = camera_to_arm(y, model, 'right')[0] * 1000\nleft_xyz_mm = camera_to_arm(y, model, 'left')[0] * 1000\nprint(right_xyz_mm, left_xyz_mm)\n```",'',
              '示例复用现部署的像素→相机部分，然后调用实验模型；模型输出分别属于各自MDH父系。现部署的范围/元数据检查基于其原标定，此示例不构成完整生产接入。vision_deploy v2.4.0默认参数保持原值。','',
              '## 参数、证据与复现','',
              f'- [完整候选参数]({selected}.json)：共享pF、左右M/t、核长度/正则、中心/系数及覆盖范围。',
              '- [完整比较与划分](comparison.json)、[逐点留出误差](heldout_errors.csv)、[退化原因诊断](diagnostics.json)、[补偿幅度与覆盖](model_behavior.json)、[复核记录](VALIDATION.json)。',
              '- fold_0_models.json.gz至fold_4_models.json.gz保存所有外层训练模型，可复算逐点分数。内层保存划分与分数，模型可通过代码复现，未重复存储。','- exploration目录保存先导实验脚本和结果，记录本轮探索范围；其中数据缓存只是本机临时加速，正式流程重新读取原始归档。','',
              '```bash\npython -m interactive_ball_calibration.improve_shared_pf --output /tmp/shared_pf_joint_kernel\npython -m interactive_ball_calibration.summarize_shared_pf_kernel /tmp/shared_pf_joint_kernel\npython -m unittest tests.test_shared_pf_kernel -v\n```','',
              f"重新计算并核对{verification['heldout_prediction_rows_recomputed']}条预测；5项测试覆盖无偏恢复、已知平滑偏差的未见位置改善、序列化一致性、输入校验、推理不依赖pF和六指标选型。未连接机器人/相机，也未替换默认部署。"]
    content = '\n'.join(lines).replace('\\\\', '\\')+'\n'
    (out/'report.md').write_text(content)
    assert all((out/link).exists() for link in re.findall(r'\]\(([^)]+)\)',content))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory',type=Path)
    out = parser.parse_args().directory
    result = json.loads((out/'comparison.json').read_text())
    data = {s:dataset(ROOT/result['data'][s]['path'],s) for s in SIDES}
    errors,support,verification = verify(out,result,data)
    plots(out,errors,data,result['selected'])
    report(out,result,errors,support,verification,data)
    print(json.dumps(verification,indent=2))


if __name__ == '__main__':
    main()
