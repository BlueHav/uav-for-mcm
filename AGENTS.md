# 执行约束
禁止本机运行或加载 ray_carvo.so、ray_servo.so、ray_carve.so；禁止本机导入 local_map 或间接导入 Runtime。所有Runtime、仿真、native测试和回放必须通过SSH在服务器执行。本机仅允许读取、编辑、同步及纯SSH工具。
本仓库是第一代单机通过基线，不包含之后未通过的双机修复。后续开发请新建分支并保留本标签。
