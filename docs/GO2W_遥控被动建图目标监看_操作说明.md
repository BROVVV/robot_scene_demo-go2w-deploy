# GO2W 遥控被动建图目标监看操作说明

## 1. 功能边界

本模式只做传感器读取、二维/三维建图、目标候选检测和按需 VLM 复核。机器狗运动只由 Unitree 实体遥控器控制；软件不发布 `/cmd_vel`、`/go2w/motion` 或 `/wirelesscontroller`，不启动自主搜索，也不接管 Nav2 行驶。

目标监看与普通语义落图分离：较低的目标候选阈值只用于触发复核，不会绕过普通对象地图的置信度、面积和生命周期门槛。VLM 超时或异常记为 `UNKNOWN`，不等同于目标不存在。

## 2. 启动

先用实体遥控器让机器狗停稳，再启动：

```bash
cd /home/mxt/robotscene
bash scripts/go2w/start_remote_control_mapping.sh \
  --detector grounded_sam --no-browser
```

YOLOE-Seg 是显式可选后端：

```bash
bash scripts/go2w/start_remote_control_mapping.sh \
  --detector yoloe_seg --no-browser
```

默认 WebUI 地址为 `http://127.0.0.1:8766`。启动输出应显示目标监看已启用、VLM daemon 是复用还是本模式所有，以及 WebUI 只读门自检通过。

## 3. 使用 WebUI

在 mapping tools 区的“被动目标监看”卡片中输入自然语言目标，例如：

- `黑椅子`
- `饮水机旁边的凳子`
- `烟雾`
- `泄漏的液体`
- `史莱姆泥`

点击“设置目标”后，页面显示解析出的目标词、锚点词、关系和属性。候选框为黄色，复核中为蓝色，确认后为绿色。有可靠拍摄时刻地图坐标时，确认目标会作为独立 marker 叠加在地图上；只有相机坐标时仍显示视觉结论，但不会伪造地图位置。

点击“清除目标”只停止新的目标复核并推进 generation，普通建图和语义落图继续运行。

## 4. 状态含义

| 状态 | 含义 |
|---|---|
| `IDLE` | 未配置目标 |
| `ARMED` | 已配置目标，等待 detector 候选 |
| `CANDIDATE` | detector 已产生候选，尚未得到 VLM 终态 |
| `VERIFYING` | VLM 正在复核 |
| `CONFIRMED` | VLM 确认目标；普通物体默认停止继续复核 |
| `REJECTED` | VLM 明确判定候选不符合目标 |
| `UNKNOWN` | 超时、异常或返回无法判定；不是负例 |
| `DEGRADED` | VLM daemon 不可用；候选仍可显示，普通建图不受阻 |

关系目标只有目标与锚点同时通过几何门时才触发 VLM，复核裁剪同时覆盖二者。没有候选时 VLM 调用数严格为 0。

## 5. API

这些接口只在遥控 mapping 的 `display_only=true` 模式可用：

```bash
curl http://127.0.0.1:8766/api/mapping/target-watch

curl -X POST http://127.0.0.1:8766/api/mapping/target-watch \
  -H 'Content-Type: application/json' \
  -d '{"target":"黑椅子"}'

curl -X DELETE http://127.0.0.1:8766/api/mapping/target-watch
```

证据图通过状态中的 `evidence_url` 读取。服务只允许读取当前状态登记且位于目标证据目录内的文件。

## 6. 无运动 smoke

普通检测链：

```bash
bash scripts/go2w/start_remote_control_mapping.sh \
  --detector yoloe_seg --smoke --no-browser --no-web
```

目标链只采一帧；有候选时最多触发一次真实 VLM，无候选时输出 `NO_CANDIDATE`：

```bash
bash scripts/go2w/start_remote_control_mapping.sh \
  --detector yoloe_seg --smoke --smoke-target '黑椅子' \
  --no-browser --no-web
```

`--smoke-target` 不能脱离 `--smoke` 使用。camera-local smoke 不启动 SLAM、WebUI、轮式里程计或激光预处理。

## 7. 停止

先用实体遥控器把机器狗停稳，再执行：

```bash
bash scripts/go2w/stop_remote_control_mapping.sh
```

停止脚本按 PID、命令行和 ownership marker 定向清理。外部复用的 SLAM 或 VLM daemon 保持运行。

## 8. 产物与排障

- 状态：`outputs/remote_control_mapping/runtime/status.json`
- 目标请求：`outputs/remote_control_mapping/runtime/target_watch_request.json`
- 目标状态：`outputs/remote_control_mapping/runtime/target_watch_state.json`
- 目标证据：`outputs/remote_control_mapping/runtime/target_watch_evidence/`
- 事件：`outputs/remote_control_mapping/runtime/events.jsonl`
- 侧车日志：`outputs/remote_control_mapping/logs/semantic_mapper.log`
- Web 日志：`outputs/remote_control_mapping/logs/web_server.log`

常见问题：

- `DEGRADED` 且提示 socket 不可用：检查 `runtime/go2w/siliconflow_vlm.sock` 和 daemon 日志；普通建图可继续。
- `NO_CANDIDATE`：这是合法结果，不会调用 VLM。关系目标还需确保目标和锚点同时出现在画面内。
- 只有 `camera_xyz`、没有 `map_xyz`：检查 SLAM、TF、`map_epoch` 和拍摄时刻位姿；系统会保持降级而不是猜坐标。
- 页面未更新：确认使用 8766 端口，并检查目标状态文件的 `mapping_session_id` 与当前 topology 会话一致。
- 电量低于启动硬门时先充电；不要通过降低门槛长期运行。
