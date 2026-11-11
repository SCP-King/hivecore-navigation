# N-01 硬件利用率快照（2026-11-11 14:00 现场）

> 结论：**不算"充分利用"，但也不是"算力闲置"**。
> 7 个非隔离核已经跑到 87–93%（2.9 GHz 定频、无降频），
> 而预留的隔离核 cpu7 只有 7% 占用；感知链只跑到标称速率的 40–60%。
> 也就是说：**总量并不宽裕，但分配方式让关键路径用不上这些算力。**

## 1. 快照

| 项 | 实测 | 判断 |
|---|---|---|
| CPU | i7-10700，8 逻辑核（8C16T 的 HT 在 BIOS 关闭，Thread(s)/core = 1），2.9 GHz `performance` 定频 | — |
| 各核占用 | cpu0 87.6% / cpu1 93.0% / cpu2 90.0% / cpu3 86.5% / cpu4 89.2% / cpu5 92.6% / cpu6 89.3% / **cpu7 7.3%** | 6.17/8 核忙，cpu7 空转 |
| load / PSI | load 13.15（5 min 12.58）；`/proc/pressure/cpu` some=30.7%，full=0；`procs_running` 16 | 非隔离核已超订 |
| 温度/降频 | 封装 55 °C，`core_throttle_count=0` | 散热没问题 |
| 内存 | 16.18 GB，available 9.74 GB，cache 5.36 GB，swap 4.0 GB **0 使用** | 富余，不是瓶颈 |
| 磁盘 | sda 自启动写入约 3.6 GB；io PSI 0 | 空闲 |
| GPU | 集显 UHD 630，i915 已加载并被桌面使用；无独显/CUDA | 有集显，但没有可用于 SLAM 的算力 |
| 网卡 | enp8s0 收 2.98 MB/s；**lo 回环 26.4 MB/s 双向**（DDS） | 回环流量偏大 |
| 丢包 | UDP `RcvbufErrors` 3,788,573 / `InErrors` 3,788,573 / `NoPorts` 1,722,764 | 约 3.3% 收包被丢 |
| 中断 | IRQ 152 有 22.1M 次在 **CPU0**；IRQ 153 在 CPU3；IRQ 151/154 被绑到隔离核 **CPU7**（各约 4.6 万次）；默认亲和 0-6 | 不均 |
| 传感器 | `/livox/lidar` 5.96 Hz（标称 10 Hz）；`/cloud` 4.16 Hz；`/n01/localization/pose` 3.76 Hz | 只跑到 40–60% |
| 执行器 | 底盘/轮足未动（operator_hold + `safety_authorization=NEVER`） | 有意为之 |
| 内核隔离 | `isolcpus=7 nohz_full=7 rcu_nocbs=7 irqaffinity=0-6 idle=poll` | 隔离了但没派活 |

当前 CPU 主要落在 `user.slice` 的 6.17 核：`session-3.scope` **4.23 核**（37 进程 / 6.6 GB）、
`user@1000.service` 1.84 核（其中一个终端 1.15 核 / 14 进程 / 0.30 GB，另一个 0.58 核）、
Firefox 0.08 核、system 0.02 核。所有 cgroup 都没有 `cpu.max` 配额；上面抽查的三个 scope 也没有 cpuset 绑核。

> 采样时 `n01_localization_far_test` 正在跑（日志 14:06 仍在写），
> 加上沙箱 PID namespace 看不到宿主进程名，**4.2 核只能定位到 scope，不能精确到进程**。
> 结合现场（`localization_lite_node` 反复 `LO scan_ref miss ... busy=true`、`/n01/localization/pose` 只有 3.76 Hz），
> 最可能是甲方 Lite 定位/SLAM/GUI 这条链。

## 2. 三个结构性问题

1. **隔离核没派活**：`isolcpus=7` 让普通任务不会被调度到 cpu7，但没有任何 cpuset/taskset 把关键线程绑上去，
   结果 load 13 时 cpu7 仍 93% 空闲。反而 IRQ 151/154 被手动绑到了这个核（量很小）。
   这是"预留了实时核但没接上"——最典型的硬件浪费。
2. **关键路径是单线程**：`scan_planner_node` 用 `SingleThreadedExecutor`，A\*（4–6 ms）和 min-snap（300–950 ms）
   都挤在一个线程；建图 raycast（80 ms/帧）也在同一线程。旁边 6 个核再忙也帮不上它，
   它自己还因为占不到 CPU 出现 4–357 ms 的 goal 排队。**不是核不够，是关键线程没有被隔离出来。**
3. **感知链欠采样**：雷达标称 10 Hz 实际 5.96 Hz，定位 3.76 Hz，`/cloud` 4.16 Hz；
   同时内核 UDP 已丢 379 万包、回环 26 MB/s。也就是说 CPU 很忙，但**忙的不全是有效数据通路**，
   传感器/网络这一侧反而没吃满。

## 3. 建议

| 优先级 | 动作 | 说明 |
|---|---|---|
| P0 | 把 scan_planner（以及底盘/控制链）绑到 cpu7：cgroup cpuset 或 `taskset -cp 7 <pid>`；同时把 enp8s0 的 IRQ 从 cpu7 挪走 | 隔离核才真正服务实时路径；否则不如去掉 `isolcpus`/`idle=poll` 把核还给调度器 |
| P0 | 查清 `session-3.scope` 那 4.2 核是谁（很可能甲方 Lite 定位/GUI），验收时降优先级或停掉非必要 GUI/far test | 给 N-01 栈留 2–3 核；现在它和 4.2 核的负载混跑 |
| P1 | DDS 走共享内存、调大 UDP `rmem`、把 enp8s0 多队列 IRQ 分摊到 0–6 | 回环 26 MB/s + 379 万丢包说明本机传输有优化空间 |
| P1 | 复查 `/livox/lidar` 为什么只有 5.96 Hz（驱动、IRQ、CPU 竞争） | 雷达不上 10 Hz，后面的定位/规划都被限速 |
| P2 | 评估 BIOS 开 HT（8C16T → 16 线程） | 能多约一倍线程吞吐，但 RT 抖动要重新测；当前是有意关闭还是漏配需要确认 |
| — | 内存、磁盘、GPU 不需要扩容 | 9.7 GB 可用、磁盘空、集显只负责桌面 |

## 4. 复现

```bash
# 每核占用（3 s 窗口）
python3 - <<'EOF'
import time
def snap():
    d={}
    for l in open('/proc/stat'):
        if l.startswith('cpu'):
            p=l.split(); d[p[0]]=[int(x) for x in p[1:]]
    return d
a=snap(); time.sleep(3); b=snap()
for k in [f'cpu{i}' for i in range(8)]:
    x=a[k]; y=b[k]; d=[j-i for i,j in zip(x,y)]; t=sum(d)
    print(k, round(100*(t-d[3]-d[4])/t,1), '%')
EOF
cat /proc/loadavg /proc/pressure/cpu
grep -E "enp8s0" /proc/interrupts
grep -E "Udp:" /proc/net/snmp | tail -1
```
