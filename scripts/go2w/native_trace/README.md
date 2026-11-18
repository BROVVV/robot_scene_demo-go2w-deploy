# 只读迁移采样器

本工具仅订阅前向净空、激光新鲜度、定位健康、几何状态和里程计，并读取指定的本地 LowState JSON。没有 SDK、运动发布者、服务或 Action 客户端；它不修改安全门或授权运动。默认运行 165 秒，上限 180 秒。

在已加载对应 ROS 环境的 shell 中构建，产物放到源码树外：

```bash
cmake -S scripts/go2w/native_trace -B /绝对路径/native_trace_build -DCMAKE_BUILD_TYPE=Release
cmake --build /绝对路径/native_trace_build -j1
```

从外部证据目录构建时，可以通过 `-DGO2W_TRACE_JSON_INCLUDE=/已校验release/external/unitree_ros2/example/src/include` 指定现有 JSON 头文件。不下载依赖、不替换 ROS/DDS。

调用者先创建新的输出目录，原子更新其中的 `phase.json`（例如 `{"phase":"baseline"}`），再运行：

```bash
/绝对路径/native_trace_build/passive_trace --output-dir /新的输出目录 --low-state /本次启动的私有目录/network_low_state.json --duration 165
```

写入 `{"phase":"finished"}` 可正常结束。已有任意目标日志时拒绝启动，不覆盖先前证据。SIGINT 也会保存终态。每两秒采样 `/proc` 的进程 CPU/RSS；采样器自己的开销也应纳入评估。前向净空沿用旧 Python 采样协议的 `NaN`、`Infinity`、`-Infinity` JSON 扩展，不能把它们替换为普通距离或 `null`。定位/几何 JSON 解析失败单独记录，不当成健康值。

`verify_trace_ros.py` 必须在 Domain 99、localhost-only 环境执行，验证五种真实 ROS 消息、非有限净空、状态 JSON、里程计、双时钟/boot 拒绝、结束清理和已有日志拒绝。Foxy 的 CycloneDDS 隔离配置需使用兼容的 loopback 配置；不要混用新 `Interfaces` 与 `ROS_LOCALHOST_ONLY` 注入的旧 `NetworkInterfaceAddress`。

```bash
ROS_DOMAIN_ID=99 ROS_LOCALHOST_ONLY=1 /usr/bin/python3 scripts/go2w/native_trace/verify_trace_ros.py --executable /绝对路径/native_trace_build/passive_trace --output-dir /新的隔离测试输出目录
```
