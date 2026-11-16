# swarm-uav

来自对应服务器目录当前工作树的精简运行版本。两个分支独立保存各自 Runtime 和 Unity Player，不互相导入代码。

## 在已配置服务器运行

```bash
git clone --single-branch -b swarm-uav https://github.com/BlueHav/uav-for-mcm.git
cd uav-for-mcm
bash setup.sh
export SWARM_RUN_DIR="$PWD/.runs/$(date +%Y%m%d-%H%M%S)"
bash bash/unity.sh
# 第二个 SSH 终端进入同一目录，并设置相同 SWARM_RUN_DIR：
bash bash/control.sh ten
```

`control.sh` 启动后保持 IDLE。启动验证覆盖 ten 模式；原脚本保留其他模式，但未在此次整理中验收。
自动完整任务：`bash bash/run_mode.sh ten "$PWD/.runs/full-$(date +%Y%m%d-%H%M%S)"`，该命令会发送仿真飞行指令。

## 依赖与内容

目标环境：Linux x86_64、ROS Noetic、NVIDIA 图形驱动与可用 X display（默认 :20）。
使用服务器已有 `/root/gpufree-data/yopo-portable/env.sh` 提供的 Conda Python、ROS 和已编译控制器/TCP 工作空间。
其他服务器须先安装这些系统依赖，再通过 `UAV_SYSTEM_ENV=/path/to/env.sh` 指定环境；不宣称任意空机器 clone 后零依赖运行。
Python依赖版本见 `code/YOPO/requirements.txt`；控制器、TCP及原生规划器源码均保留在 `code/` 下。
保留模型权重、地图、CPU/CUDA本机库与对应 Linux Unity Player 压缩分片（setup自动校验、解压）。
Unity C# 源码位于 unity/Project；不包含完整编辑器美术资源工程、历史结果、训练日志或重复快照；运行输出留在 `.runs/`。

`SOURCE.json` 记录来源。`VALIDATION.md` 记录此次整理后的实际检查范围。
Windows 本机禁止导入 Runtime、local_map 或加载 ray_carve/ray_carvo/ray_servo；所有验证须在 SSH 服务器执行。
