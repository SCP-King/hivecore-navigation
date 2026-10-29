# N-01 下一阶段：动态障碍候选跟踪与短时预测（功能开发）

日期：2026-10-29。**离线研究型原型，尚未在用户机器上执行本次新增模块验收；不得连接底盘。**

## 实际数据流

1. 原有真实 Bag 白名单读取器只读获取历史 TF 与 Livox 过滤点云，内部转换为 map 坐标的 /cloud 以及 /body_pose。
2. 新增 n01_offline_dynamic_tracker 订阅 map 点云和机器人位姿，按消息时间戳做相容性检查。
3. 在机器人附近的有限范围内对 XY 栅格进行连通域聚类，将跨帧相邻聚类关联为候选障碍，估计二维速度并给出 0.5、1.0 和 1.5 秒的匀速预测。
4. 通过 /n01/offline/dynamic_tracks（std_msgs/String JSON）发布目标假设、位置、速度、预测点及处理统计。
5. n01_dynamic_port 消费动态候选和 SCAN 的路径参考，发布 /n01/offline/dynamic_port_status；现阶段 Kalman 与 MPC-CBF 仍是 NOT_IMPLEMENTED，actuator_output 为 DISABLED。
6. 原有 SCAN 三维规划、B 样条、Path 桥接继续并行工作；只有观察与报告，没有真实运动指令。

## 验证界限

- 真实 Bag 可能只有静态场景，移动障碍候选数为 0 是正常情况，并不意味着算法失效。
- 运动假设没有语义类别或人工真值标注；点云抖动、遮挡和聚类拆分可能造成误判。**不能声称已经完成真实药厂动态避障。**
- 原有旧 Bag 的 /localization_status 依然缺失，TF 高度历史冲突依然存在；此模块运行在历史开环重建后的 map 数据上，而不是严格实时定位门控模式。
- 已新增不依赖 ROS 的合成单元测试：静态目标保持静止、移动目标预测、机器人位移情况下不误判、时间倒退重置、过期目标处理、错误数据拒绝、输出无控制权限。
- 目标仅为功能打通。当前不调整规划速度、碰撞裕量、时间预算或药厂安全距离，性能优化后续开展。

## 一键检查

在真机控制断开时，继续使用已经跑通的真实 Bag 一键脚本：

    cd /home/demo/lyy/hivecore-navigation && git -c http.version=HTTP/1.1 pull --ff-only origin main && bash scripts/run_n01_bag_functional.sh

上传输出的 n01_bag_functional_*.tar.gz。需重点查看 recorded_bag_report.json 的 counts.dynamic_track_processed 是否大于 0，以及 dynamic_port.dynamic_processed_frames_seen 是否大于 0。
moving_hypothesis_frames 可以为 0，不因此判 FAIL；/cmd_vel 发布者必须为 0，mpc_cbf_solver 仍为 NOT_IMPLEMENTED。新增功能需要用户运行后才能判断是否通过。

## 下一步

对有真值的运动目标确认速度估计和误报，再开发不确定性建模及轨迹级的离线 CBF/MPC 约束评估器；正式控制与安全验收仍禁止越级。

参考：docs/n01_real_bag_functional.md；docs/hive2_slam_p0_readiness_preflight.md。
