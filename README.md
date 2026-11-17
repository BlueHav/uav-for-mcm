# swarm-uav-combine

YOPO 独立末段飞行版本。Unity 运行包、权重、地图和控制器/TCP 源码均随分支提供；不读取另一个 swarm-uav 项目的 Unity、Runtime、环境脚本或已编译工作空间。

**验证状态：平稳飞行与终点编队接管版本完整通过。** `full-smooth-star-05` 从第一阶段连续运行，十架全部完成编队接管、星型触地停桨，碰撞及越界为零；121 项服务器回归通过。相比第一版，航路速度总变差减少 60.1%，近零速进入次数减少 77.4%，平均航路速度提高 3.3%。总任务约 22 分 17 秒，比第一版多约 33 秒，不宣称全流程更快。详见 [VALIDATION.md](VALIDATION.md) 和[实际触地站位](docs/validation/star-touchdown.png)。

第一版成功代码永久保留在标签 **`combine-v1-success`（8158346）**。终点接管已实现，但本轮进入 30 m 范围至触地的等待时间没有缩短，不能宣称终点阶段更快。距离/队形调速最低 2 m/s，局部绕障允许 1.5 m/s；必要安全制动和终点停车保留。50 m 不作为本轮到达验收条件，也不作硬上界保证。

## 安装与独立构建（Linux 服务器）

系统依赖：Ubuntu 20.04 / ROS Noetic、Python 3.8、C++ 编译器、Eigen、Boost、ROS 所需开发库、NVIDIA 图形驱动和可用的 X display。GPU 推理还需要与 PyTorch 匹配的 CUDA 驱动。系统 ROS/Python/CUDA 安装不属于另一个项目的依赖。

```bash
sudo apt-get install build-essential cmake python3-venv python3-psutil \
  libeigen3-dev libboost-dev libboost-system-dev libboost-thread-dev \
  libboost-filesystem-dev libboost-regex-dev libboost-chrono-dev \
  libboost-date-time-dev liblog4cxx-dev libconsole-bridge-dev \
  liblz4-dev libbz2-dev libtinyxml2-dev libtinyxml-dev patchelf
```

```bash
git clone --single-branch -b swarm-uav-combine https://github.com/BlueHav/uav-for-mcm.git
cd uav-for-mcm
python3 -m venv --system-site-packages .venv
.venv/bin/python -m pip install -r code/YOPO/requirements.txt
bash setup.sh
```

如已装好推理环境，可用 `export YOPO_PYTHON=/path/to/python` 指定，两个运行终端使用相同设置。`setup.sh` 在当前目录构建自己的控制器和 TCP 工作空间，校验、解压本分支 Unity 程序，编译 CPU 地图库并移除动力学二进制的旧 RPATH。

## 运行

先停止前一会话。图形会话通过 `SWARM_DISPLAY`、`SWARM_XAUTHORITY` 配置，不在代码中固定用户目录。例如本次测试服务器使用 display `:20`。

```bash
export SWARM_RUN_DIR="$PWD/.runs/$(date +%Y%m%d-%H%M%S)"
echo "$SWARM_RUN_DIR"
bash bash/unity.sh
# 第二个 SSH 终端进入同一目录，复制上面输出的 SWARM_RUN_DIR：
export SWARM_RUN_DIR=/absolute/path/from/first/terminal
bash bash/control.sh ten
```

`control.sh` 启动后保持 IDLE。启动验证覆盖 ten 模式；原脚本保留其他模式，但未在此次整理中验收。
自动完整任务：`bash bash/run_mode.sh ten "$PWD/.runs/full-$(date +%Y%m%d-%H%M%S)"`，该命令会发送仿真飞行指令。本次完整验证使用该正式入口，没有跳过阶段或改写起飞站位。

## 飞行与验证

真实领航机为 2、7。最终收拢、垂直落地前和触地后的站位使用 Swarm-Formation 的 `final` 模板：领航机在前，两翼各两架；高空并队的原十字交接入口保持不变，避免改变旧轨迹的成员身份映射。末段配置限速 6 m/s，实际峰值、触地误差和碰撞记录须以验证报告为准。

每组五架均进入各自目标的 30 m 范围后，先受控减速并核验自身观测、完整进场走廊、落地竖直柱和机间净距。全员新鲜确认后，统一切换到编队轨迹，停止 YOPO 推理和主动绕障规划；名义轨迹最高 2.5 m/s，跟踪指令最高 3 m/s，保留原有动力学积分与紧急保护。未通过准入时继续 YOPO，不能穿越未确认的走廊。只有五架都确认接管、轨迹执行完毕且星型稳定后，才允许进入降落。

航路使用缓慢恢复速度、方向选择滞回和与最终停车检查一致的较低速度候选，减少加速后又被下游否决的循环。完整停车走廊不安全时仍可制动，因此“更平稳”不代表全程无波动或无安全停车。

距离限制位于 `config/point.json` 的 `terminal_cohesion`：XY 相邻距离以 50 m 为目标上限，25 m 内解除距离滞回。3.5 s 趋势预测及 22 m 制动预留用于提前减速。按候选速度是否正在远离相邻机判断该减速的成员，水平速度仅等比例缩放，编队控制不修改航向、绕障点或终点。

距离及队形调速的上限保持在 **2–6 m/s**。队形系数复用上游 Swarm-Formation 的归一化 Laplacian 差平方和，以 XY 坐标计算；日志提供各行贡献和逐机归位后的误差改善量，辅助定位偏差成员。系数对整体缩放不敏感，另用模板间距和航向投影检查前后层级：二排不得超领航机，三排不得超对应二排。已超前的后机降到 2 m/s，前级优先恢复正常权限以追上。该归因是几何诊断，不能仅凭一个系数唯一推断故障责任。

各机仍独立使用 YOPO 绕障，局部绕障允许 1.5 m/s；无已观测安全走廊时保留安全制动与扫描，传感器、邻机保护、操作停止及末端落地允许低于 2 m/s。距离/队形调速器本身不将权限降到 1.5 m/s，也不把原本安全需求低于 2 m/s 的速度强行提高。若后机安全速度低于 2 m/s，前机继续前进至少 2 m/s 时距离仍可能增加；此前第六阶段专项的严格距离审计失败。前后级限速和最终顺序恢复已观察到，不能保证全程不超前。见 [VALIDATION.md](VALIDATION.md)。

| 组 | 领航机 | 两翼前后级相邻关系 |
| --- | --- | --- |
| 0 | 2 | 2—0、2—1、0—3、1—4 |
| 1 | 7 | 7—5、7—6、5—8、6—9 |

关系来自原有 final 星型模板，模板两类相邻距离约 6.71 m、4.24 m，并记录在遥测中。50/25 m 是这些树边的水平距离门限，末排到领航机经过两级相邻关系。每组独立维护滞回状态；碰撞、传感器与停车检查继续生效。

聚焦末段的真实仿真测试（会实际起飞、飞行和落地）：

```bash
source env.sh
/usr/bin/python3 tests/run_phase6.py --run "$PWD/.runs/phase6-$(date +%Y%m%d-%H%M%S)"
```

该测试在两组低空交接位置从地面起飞，运行正式 CLOUD→YOPO 交接、独立规划与垂直落地；它不代表前五阶段或全任务验收。报告与故障地图快照写入此次 `.runs/.../Reports/`。

距离控制的干预验证用 `--lag-seconds 20 --lag-speed 2`：测试夹具在 YOPO 启动后将第一组末排 3 号机的速度权限限制为 2 m/s，持续 20 s，随后恢复正常权限。正式 Runtime 没有注入开关。用 `tests/audit_phase6.py --run RUN --require-cohesion --require-lag --require-formation` 核查实际减速、追近、25 m 内恢复、再次加速、队形系数、2–6 m/s 权限与超前成员减速，以及全队最终星型落地。`--require-distance-cycle` 额外要求实际触发并解除距离滞回；若队形层已提前保持在门限以内，审计会明确记录未触发。距离上限检查始终保留。

飞行结束后在服务器运行 `/usr/bin/python3 tests/audit_phase6.py --run /absolute/path/to/run`，核对十机实际推理、各机地形来源、垂直落地前星型站位、触地与电机停转状态及 Unity 事件。

完整验证使用以下监督器，它内部调用上述正式入口，记录运行指纹与结果，并在结束后清理本次进程。它与本次服务器测试使用的监督器一致：

```bash
export RUN="$PWD/.runs/full-$(date +%Y%m%d-%H%M%S)"
/usr/bin/python3 tests/run_full_mission.py --project "$PWD" --run "$RUN"
source env.sh
/usr/bin/python3 tests/audit_full_mission.py --run "$RUN"
# 与保留的第一版运行日志比较平稳性和全员接管：
"$YOPO_PYTHON" tests/audit_smooth_terminal.py --run "$RUN" --baseline /absolute/path/to/full-pipeline-02
```

日志位于 `Reports/ten-runs/运行ID`。完整审计额外核查两组从起飞、组队、爬升、巡航、下降、低空重排到独立 YOPO 的完整阶段顺序，以及十机实测巡航证据；不会把第六阶段专项夹具当作全流程通过证据。

路径失败分析见 [docs/path-failure-analysis.md](docs/path-failure-analysis.md)。
Unity C# 源码位于 unity/Project；不包含完整编辑器美术资源工程、历史结果、训练日志或重复快照；运行输出留在 `.runs/`。

安装包包含原 Player 和经过 SHA-256 校验的脚本程序集覆盖包，`setup.sh` 会安装两部分。需要修改 C# 时，可在服务器用 `python3 bash/build_unity_scripts.py --editor /path/to/Editor/Unity` 重新编译脚本；该命令使用编辑器发行包自带的 Roslyn，不启动编辑器。完整场景重建仍需要完整资源工程和有效许可证。旧 `Runtime/start_unity.sh` 仅保留分终端兼容入口；正式运行使用上述 `bash/` 脚本。

`SOURCE.json` 记录来源。`VALIDATION.md` 记录此次整理后的实际检查范围。
Windows 本机禁止导入 Runtime、local_map 或加载 ray_carve/ray_carvo/ray_servo；所有验证须在 SSH 服务器执行。
