# 2026-10-30 16:51 历史 Bag G3.4 验收审计

数据来源：用户上传的 `n01_bag_functional_20261030_165128_1541406.tar.gz`，非重播/非在线机器人测试。Git 提交：`fa7ca35`；本报告直接对上传包内的 `summary.txt`、`recorded_bag_report.json`、`g3_evidence_summary.json` 和 104 行 `g3_frame_evidence.jsonl` 复核。实机控制链物理隔离、报告指令输出禁用；不能将这些数字用作真实机器人运动学或避障安全认证。

## 本次验收结论

**G3.4 离线 XY 平移限幅：通过；动态 CBF 的真实干预效果：未由这份 Bag 证明。**

- Runner result `BAG_FUNCTIONAL_PATH_SMOKE_PASS`，exit_code=0，owned process cleanup=CLEAN，post-cleanup ROS graph=CLEAN，`/cmd_vel` publishers=0。
- 928 个历史 `/livox/lidar/geo_filtered` 点云全部处理：571 个成功到 map，357 个因为采集时刻 TF 不可用明确拒绝。**没有**在线 LocalizationStatus tracking 验收。
- SCAN 4 个有效 B 样条。G3 104 次求解，记录 104 行有完整时间戳的证据。**其中 POST-HOC 103 次，回放顺序 1 次**；不是 104 次真正在线闭环。
- 104 个 G3 候选均有 `hive2_motion_limit_screen`，对应相同 Hive2 运动参数 SHA-256 `3f19abe0a65e02b331a0d51c18ec0503278a4d9a6b942332b47c4f7d434584f0`。按筛查器实际采用的**四位小数轨迹容差**，Hive2 底盘配置超限 = 0、SCAN 配置超限 = 0。帧内最大离散加速度约 **0.50277 m/s²**，相对配置的 0.5m/s²有约 0.00277m/s² 数值差（四位小数离散化，容差 0.005），所以**不是数学上严格 <=0.5 的证据**。最大段速度约 0.75034m/s（速度容差 0.001）。
- 104 轨迹的 G3 自身数值状态：88 条 `FEASIBLE_NOMINAL_ONLY_NO_DYNAMIC_CONSTRAINTS`，16 条 `FEASIBLE_CBF_CONSTRAINED_CANDIDATE_UNVERIFIED`，没有报告 `INFEASIBLE`。其中 `FEASIBLE_` 是数值代理可行，绝非四轮转向可执行。

## 动态 CBF 标签必须谨慎解读

**16 条**有 `moving_constraint_tracks>0`，且**全部**是 POST-HOC。对其共做 **64,288 次**离散 CBF 条件检查，`cbf_constraints_rejected=0`；每条的 `minimum_nominal_h_m2 > 0`。因此只能说**检测到符合条件的移动假设并进行了 CBF 判定**；没有证据显示因 CBF 发生候选淘汰，更没有验证避障成功。所有轨迹 `trajectory_changed=true` 可能是因新的加速度格和名义参考速度不同，不能归因于 CBF 避让。

为防止错误统计，仓库已新增 `cbf_candidate_pruning_observed`（CBF 导致搜索候选淘汰）和 `nominal_cbf_proxy_overlap_observed`（未约束名义代理轨迹出现 h<0）两个独立标签；`g3_frame_evidence` 汇总分别记下动态假设、实际 CBF 候选淘汰与名义代理重叠帧数。对应测试 `test_g35_cbf_attribution.py`。**新增字段不会回写已经录好的原始 Bag，也不表示本次 Bag 中发生了新的避障干预。**

## 阶段决策

**可以结束第 1 轮 G3.4 平移加速度约束验收，不再原样重复 Bag 测试。** 但还不能宣布四轮转向执行可行、动态障碍成功规避、全身碰撞或药厂地面通行验收。

下一步进入 G4 地面/路线阶段：使用同次建图 PCD、Bag 起点的 8.036m/29 点 XY 研究路线，按每个路点的实测表面多层分布与未知区进行风险分级；不从 Z 低点推断地面、不擅自填 `ground_z_map`，没有完整支撑/整机净空证据以前不将 XY 路线放行到 SCAN。之后再用独立真值场景检查真正动态避障干预和跟踪能力。
