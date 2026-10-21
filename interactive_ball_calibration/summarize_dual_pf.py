"""Create a reviewable report/plots from saved dual-pF comparisons without refitting."""
import argparse
import csv
import json
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

from .compare_dual_pf import SIDES, dataset, flange_delta
from .compare_pro_calibration import camera_to_arm
from .pro_capture import ROOT, digest, mdh_rows
from .pro_fit import metrics
from .historical_reference import independent_rigid_reference


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory',type=Path)
    args=parser.parse_args(); directory=args.directory
    result=json.loads((directory/'comparison.json').read_text())
    data={s:dataset(ROOT/result['data'][s]['path'],s) for s in SIDES}
    labels=list(result['candidates'])
    chosen=result['selected_by_full_cv']['shared_rigid']
    chosen_models=result['candidates'][chosen]['candidate']['models']
    baseline=result['candidates']['rigid_l2/independent']['candidate']['models']
    baseline_pf={s:np.asarray(baseline[s]['flange_to_ball_center_m']) for s in SIDES}
    diagnostics=dict(pf_difference_right_minus_left_mm=((baseline_pf['right']-baseline_pf['left'])*1000).tolist(),
                     pf_difference_norm_mm=float(np.linalg.norm(baseline_pf['right']-baseline_pf['left'])*1000),
                     mdh_rows_numerically_equal=bool(np.array_equal(mdh_rows(ROOT/'configs/1004.ini',0),mdh_rows(ROOT/'configs/1004.ini',1))),
                     source_summary_script_sha256=digest(Path(__file__)),
                     deployment_manifest_sha256=digest(ROOT/'vision_deploy/stereo_robot/calibration/manifest.json'),
                     deployment_modified=False,per_arm={},fold_pf={},independently_verified=False)
    for s in SIDES:
        clouds=data[s]['clouds'];centers=np.median(clouds,axis=1)
        jitter=np.percentile(np.linalg.norm(clouds-centers[:,None,:],axis=2)*1000,95,axis=1)
        diagnostics['per_arm'][s]=dict(valid_positions=len(centers),valid_frames=int(np.prod(clouds.shape[:2])),
            within_position_3d_jitter_p95_mm=dict(median=float(np.median(jitter)),p95=float(np.percentile(jitter,95)),maximum=float(jitter.max())),
            camera_depth_m=[float(centers[:,2].min()),float(centers[:,2].max())])
    for name in ('rigid_l2/independent','rigid_covariance/shared','affine/near_0p5mm'):
        diagnostics['fold_pf'][name]={}
        for s in SIDES:
            values=np.array([f['candidates'][name]['models'][s]['flange_to_ball_center_m'] for f in result['folds']])*1000
            diagnostics['fold_pf'][name][s]=dict(min_mm=values.min(0).tolist(),max_mm=values.max(0).tolist(),
                std_mm=values.std(0).tolist(),note='Overlapping training folds; descriptive stability, not independent confidence intervals.')
    # Existing deployed right anchor remains available as an explicitly conditional comparison.
    # This is an in-sample left fit with fixed deployed right pF; do not label it held-out accuracy.
    from .registration import rigid_fit
    fixed_pf=np.asarray(independent_rigid_reference()['right']['flange_to_ball_center_m'])
    poses=data['left']['poses'];arm_points=poses[:,:3,3]+np.einsum('nij,j->ni',poses[:,:3,:3],fixed_pf)
    R,t=rigid_fit(arm_points,data['left']['points'])
    fixed_delta=(data['left']['points']-t)@R-arm_points
    diagnostics['left_fixed_to_deployed_right_pf_fit_only']=dict(pf_m=fixed_pf.tolist(),metrics=metrics(fixed_delta),
        note='Conditional left in-sample fit only; right calibration fitted on full right data, not a joint cross-validation result.')
    # Compare the candidate camera maps to the currently deployed geometric maps.
    deployed=independent_rigid_reference()
    diagnostics['shared_rigid_candidate_shift_from_deployed']={}
    for s in SIDES:
        T=np.asarray(deployed[s]['camera_from_mdh_T']);observed=data[s]['points']
        old=(observed-T[:3,3])@T[:3,:3]
        new=camera_to_arm(observed,chosen_models[s])
        diagnostics['shared_rigid_candidate_shift_from_deployed'][s]=metrics(new-old)
    (directory/'diagnostics.json').write_text(json.dumps(diagnostics,indent=2,allow_nan=False)+'\n')
    # Coordinates refer to the re-estimated candidate right frame, not the deployed anchor.
    T={s:np.asarray(chosen_models[s]['arm_to_camera_T_before_depth_mapping']) for s in SIDES}
    camera_to_right=np.linalg.inv(T['right'])
    coordinates=[]
    for s in SIDES:
        right_from_arm=camera_to_right@T[s]
        for row,pose,point in zip(data[s]['rows'],data[s]['poses'],data[s]['points']):
            pose_right=right_from_arm@pose
            ball=camera_to_right[:3,:3]@point+camera_to_right[:3,3]
            flange=ball-pose_right[:3,:3]@chosen_models[s]['flange_to_ball_center_m']
            delta=flange-pose_right[:3,3]
            entry=dict(arm=s,waypoint_index=row['waypoint_index'],fit_error_mm=float(np.linalg.norm(delta)*1000))
            for prefix,values in (('visual_ball',ball),('visual_flange',flange),('fk_flange',pose_right[:3,3])):
                entry.update({f'{prefix}_{axis}_m':float(value) for axis,value in zip('xyz',values)})
            coordinates.append(entry)
    with (directory/'shared_rigid_candidate_coordinates.csv').open('w',newline='') as stream:
        writer=csv.DictWriter(stream,fieldnames=list(coordinates[0]),lineterminator='\n');writer.writeheader();writer.writerows(coordinates)
    # Plot saved held-out errors, never fitted residuals.
    fig,axes=plt.subplots(1,2,figsize=(14,10),sharey=True)
    y=np.arange(len(labels))
    for ax,metric,title in zip(axes,('rms_mm','p95_mm'),('Held-out flange RMS (mm)','Held-out flange P95 (mm)')):
        for offset,s,color in ((-.17,'right','#1f77b4'),(.17,'left','#e07a24')):
            ax.barh(y+offset,[result['candidates'][n]['cv'][s][metric] for n in labels],height=.32,label=s,color=color)
        ax.axvline(1,color='crimson',linestyle='--',label='1 mm');ax.set_title(title);ax.set_xlabel('mm');ax.grid(axis='x',alpha=.2)
    axes[0].set_yticks(y,labels);axes[0].invert_yaxis();axes[1].legend();fig.tight_layout();fig.savefig(directory/'comparison.svg');plt.close(fig)
    lines=['# 双臂同型黄球连接件：共享pF重标定与误差分析','',
           '日期：2026-10-20。用户确认两侧连接件同尺寸、同定位安装方向。本次检查远程仓库至 `2c073de`，没有新增采集；使用最新右臂288点和左臂424点。', '',
           '**结论：21个组合均未达到双臂留出P95≤1mm。两臂共享pF是合理的待验证物理约束，但强制共享后，当前刚体模型的数据一致性下降。当前部署v2.4.0与固定右臂锚保持原结果，本次参数另存为可检查候选。**','',
           '## 数据与物理假设','',
           '- 右臂360条记录中288有效/72拒绝，14400帧；左臂482条中424有效/58拒绝，21200帧。拒绝点未重新纳入，没有按拟合残差删点。',
           '- 校验归档清单哈希/字节数、相机和MDH来源、帧唯一性、50帧融合及原生FK。相机内参/畸变/双目几何与MDH保持固定。',
           '- 两臂MDH数值行相同，重力/安装方向配置不同；同型连接件同定位是用户提供的物理信息，并不独立证明关节零位、法兰实际坐标轴和模型完全准确。',
           '- 若所有坐标定义与安装完全一致，pF应相同或很接近。各臂独立估计的差异也可能吸收了系统偏差，不能直接解释为连接件制造误差。',
           f"- 原独立pF之差（右−左）约 {np.round(diagnostics['pf_difference_right_minus_left_mm'],6).tolist()} mm，向量长度 **{diagnostics['pf_difference_norm_mm']:.6f} mm**。",'',
           '## 方法与验证','',
           '七种模型（普通刚体、Huber、协方差加权、像素重投影、尺度、逆深度偏差、仿射）各配三种pF策略，共21组合。',
           '', '| pF策略 | 约束 |','|---|---|','| independent | 各臂独立估计3维pF |','| shared | 两臂使用同一个3维pF |','| near_0p5mm | 共用中心偏置±差异/2，每个差异分量限定±0.5/√3mm，因此向量差≤0.5mm |','',
           '0.5mm是本次敏感性分析的安装差异假设，不是实测公差。采用的是球内的轴对齐盒约束，并非在整个0.5mm球面上搜索。所有near组合最终都触及差异约束，说明在该模型/数据下仍倾向于更大的pF差异。', '',
           '共享/near拟合同时估计两臂各自外参；本次新候选的右外参不是原固定锚。每个有效位置贡献一次残差，左臂点数更多，自然具有更多观测；协方差加权还按窗口噪声分配权重。', '',
           '外层5折：两臂各按轨迹顺序留出对应区块，两侧剔除相邻2个有效位置。内层4折只在外层训练数据中选择方案。两臂测试区块均不参与训练；每个位置的50帧不跨训练/验证。', '',
           '选型优先最小化两臂中较大的P95，再比较较大的RMS；同时分别记录全部模型、仅刚体、仅共享pF刚体三个选型组。没有把左右臂合并成一个平均分掩盖较差的一侧。', '',
           '误差直接按 **相机观测经模型逆映射→减去FK旋转后的pF→视觉法兰原点，与FK法兰原点相减** 计算，单位mm。尺度/逆深度/仿射使用完整逆映射，因此这些分数与旧报告只在相机域评价的数值略有差异。', '',
           '## 21组合的留出结果','',
           '| 模型 / pF策略 | 右RMS | 右P95 | 右最大 | 右≤1mm | 左RMS | 左P95 | 左最大 | 左≤1mm |','|---|---:|---:|---:|---:|---:|---:|---:|---:|']
    for name,record in result['candidates'].items():
        a,b=[record['cv'][s] for s in SIDES]
        lines.append(f"| {name} | {a['rms_mm']:.3f} | {a['p95_mm']:.3f} | {a['max_mm']:.3f} | {a['fraction_within_1mm']:.2%} | {b['rms_mm']:.3f} | {b['p95_mm']:.3f} | {b['max_mm']:.3f} | {b['fraction_within_1mm']:.2%} |")
    lines+=['','![双臂留出误差](comparison.svg)','','RMS<1mm只表示平方平均误差小于1mm，不能解释为95%或全部点都小于1mm。最佳单次选型分数仍可能乐观，下面单列内外层隔离的嵌套流程指标。','','## 嵌套选型的泛化估计','','| 选型范围 | 右RMS | 右P95 | 右≤1mm | 左RMS | 左P95 | 左≤1mm |','|---|---:|---:|---:|---:|---:|---:|']
    for group,values in result['nested_selection'].items():
        a,b=[values[s] for s in SIDES]
        lines.append(f"| {group} | {a['rms_mm']:.3f} | {a['p95_mm']:.3f} | {a['fraction_within_1mm']:.2%} | {b['rms_mm']:.3f} | {b['p95_mm']:.3f} | {b['fraction_within_1mm']:.2%} |")
    lines+=['','嵌套分数评价每折在训练数据上选型的流程，不是单个全量模型的绝对精度。只有单会话/臂的有效位置，仍缺少独立位置真值。','','## 可检查的参数候选','']
    for group,name in result['selected_by_full_cv'].items():
        record=result['candidates'][name];candidate=record['candidate'];folder=name.replace('/','__')
        lines += [f'### {group}：{name}','',f"- [右臂参数]({folder}/right.json)，[左臂参数]({folder}/left.json)。"]
        for s in SIDES:
            pf=np.array(candidate['models'][s]['flange_to_ball_center_m'])*1000
            lines.append(f'- {s} pF（本臂法兰局部，mm）：`{pf.tolist()}`。')
        if candidate['method']=='affine':lines.append('- 该候选含尺度/剪切，不能作为刚体R直接放入vision_deploy；它并未达到双臂P95≤1mm。')
        if group=='shared_rigid':lines.append('- 该候选保持刚体形式且pF完全相同，但验证尾部误差高于原独立刚体。保存外参和对齐供复查，不替换现有部署。')
        lines.append('')
    lines+=['## pF一致性与误差来源分析','',
            '1. **纯共享约束没有消除系统不一致。** 普通刚体共享解是两套数据的折中；相同几何先验不能凭空修正相机、关节零位、柔性或安装坐标定义误差。',
            '2. **姿态与偏置耦合仍需检查。** 独立pF在连续留出折中的范围列于下表；这些折的训练集高度重叠，范围/标准差只是稳定性描述，不能当作置信区间。', '']
    lines+=['| 独立刚体pF | 全量XYZ mm | 五折训练解最小XYZ mm | 最大XYZ mm |','|---|---|---|---|']
    for s in SIDES:
        stat=diagnostics['fold_pf']['rigid_l2/independent'][s]
        lines.append(f"| {s} | {np.round(baseline_pf[s]*1000,4).tolist()} | {np.round(stat['min_mm'],4).tolist()} | {np.round(stat['max_mm'],4).tolist()} |")
    lines+=['', '3. **帧内抖动与系统残差不同。** 下表是每位置50帧相对其三维中位数的95%半径分布，不能拿它替代绝对精度：','',
            '| 臂 | 窗口P95半径的中位数 mm | 窗口P95半径的95分位 mm | 最大 mm |','|---|---:|---:|---:|']
    for s in SIDES:
        jitter=diagnostics['per_arm'][s]['within_position_3d_jitter_p95_mm']
        lines.append(f"| {s} | {jitter['median']:.3f} | {jitter['p95']:.3f} | {jitter['maximum']:.3f} |")
    lines+=['',
            '4. **共享约束需改变外参才能成立。** 下表比较共享刚体候选与当前部署对同一相机点输出的MDH坐标差，仅表示参数更新幅度，不是真值误差：','',
            '| 臂 | 坐标变化RMS mm | P95 mm |','|---|---:|---:|']
    for s in SIDES:
        stat=diagnostics['shared_rigid_candidate_shift_from_deployed'][s]
        lines.append(f"| {s} | {stat['rms_mm']:.3f} | {stat['p95_mm']:.3f} |")
    fixed=diagnostics['left_fixed_to_deployed_right_pf_fit_only']['metrics']
    lines+=['',f"5. **沿用现有右pF直接约束左臂并非更优。** 固定右臂现有pF，只重估左R/t，左臂训练RMS={fixed['rms_mm']:.3f}mm、P95={fixed['p95_mm']:.3f}mm。这只是条件训练误差，不能称为独立留出验证。",'',
            '现有数据不能唯一定位误差来源。可能因素包括球轮廓到球心修正（40mm直径假设）、相机残余畸变/视差偏差、两场采集间安装变化、机械臂关节零位与实际MDH偏差、受力挠曲和法兰局部坐标定义。拟合更灵活的仿射矩阵可吸收其中一部分，但并未证明物理原因，也未使P95达到1mm。', '',
            '## 若要求大多数位置小于1mm，下一步怎么采','',
            '- 独立测量连接件法兰定位面/原点到球心的三维偏置，同时核对法兰局部轴向；“同型同定位”提供一致性先验，不能替代该实测。',
            '- 在重叠工作空间内，两臂分别从多种末端姿态到达多个共同球心位置；同时包含近似固定姿态的平移采样，帮助区分pF、外参和姿态相关误差。记录受力/负载与相机安装状态。',
            '- 保留独立日期或独立轨迹作为最终验收集，验收前不再用于调参；同时报告左右臂三维P95、最大值和≤1mm比例。',
            '- 对真实业务目标使用独立已知位置基准测试。黄球换算法兰用到pF及FK姿态，而普通目标实时推理不用它们；本次结果不能保证任意物体单帧定位精度。', '',
            '## 文件与复现','',
            '- [软件测试与结果复核](VALIDATION.json)：合成恢复测试、全部留出模型逆算、无泄漏划分与导出校验。',
            '- [完整结果及每折模型](comparison.json)：21组合、内层分数、外层划分、全量模型、嵌套指标与来源清单。',
            '- [逐点留出法兰误差](heldout_flange_errors.csv)：原生MDH坐标下XYZ残差，保留所有有效位置。',
            '- [共享刚体候选坐标对照](shared_rigid_candidate_coordinates.csv)：两臂统一到**本次重新估计的候选右MDH参考**，含视觉球心、视觉法兰和FK法兰；该表是全量拟合对照，不是留出误差。',
            '- [诊断与部署哈希](diagnostics.json)：pF差异、折间稳定性、帧内抖动与部署保持原值的记录。',
            '- 每个 `方法__策略/` 子目录包含左右参数；刚体候选另含 `alignment.json`。同一候选的左右文件必须成对使用。', '',
            '在仓库根目录执行，输出目录必须不存在：','',
            '```bash','python -m interactive_ball_calibration.compare_dual_pf --output /tmp/dual_pf_comparison',
            'python -m interactive_ball_calibration.summarize_dual_pf /tmp/dual_pf_comparison',
            'python -m unittest tests.test_dual_pf -v','```','',
            '未连接机器人/相机，未执行运动，未修改vision_deploy参数，也未声称1mm绝对精度验收通过。']
    (directory/'report.md').write_text('\n'.join(lines)+'\n')
    print(json.dumps(dict(shared_rigid_candidate=chosen,diagnostics=diagnostics),indent=2))


if __name__=='__main__':main()
