# YOLOE-Seg 部署与切换

本项目把 `yoloe_seg` 作为额外的本地开放词汇实例分割后端。现有
`grounded_sam` 保持默认值；选择 YOLOE 后，依赖、权重、CUDA 或文本编码器
不可用都会显式失败，不会切换到 Grounded-SAM、VLM 或 CPU。

## 当前锁定版本

- Python 环境：`/home/mxt/robotscene/.yoloe_env`
- 模型：`models/yoloe-26s-seg.pt`
- 文本编码器：`mobileclip2_b.ts`
- 版本与 SHA256：见 `models/yoloe_manifest.json`
- Python 依赖：见 `requirements-yoloe.txt`

环境和大权重均为本机运行资产，已从 Git 排除；manifest 和 requirements 应纳入
版本控制。迁移机器时先从受信任的制品源放入两个权重文件，再核对：

```bash
sha256sum models/yoloe-26s-seg.pt mobileclip2_b.ts
cat models/yoloe_manifest.json
```

如需重建隔离环境，可从项目已验证的 Conda 环境克隆，然后安装锁定依赖：

```bash
conda create -y -p "$PWD/.yoloe_env" --clone go2_robot_scene_demo
.yoloe_env/bin/python -m pip install -r requirements-yoloe.txt
```

运行参数统一从 `.env` 读取，样例见 `.env.example`。模型必须是支持动态文本
prompt 的 `*-seg.pt`，不能换成 `*-seg-pf.pt`。

## 无运动检查

遥控建图的一帧 smoke 只读取 RGB-D，不启动软件运动链：

```bash
bash scripts/go2w/start_remote_control_mapping.sh --detector yoloe_seg --smoke
```

正式被动语义建图由操作员使用实体遥控器驾驶；程序本身不发运动指令：

```bash
bash scripts/go2w/start_remote_control_mapping.sh --detector yoloe_seg
bash scripts/go2w/stop_remote_control_mapping.sh
```

也可设置 `REMOTE_MAPPING_DETECTOR=yoloe_seg`，但命令行 `--detector` 优先级最高，
其次是环境变量，最后是 `configs/go2w/semantic_mapping.yaml`。运行状态、实际后端、
模型 hash、设备、精度、阈值、prompt hash、延迟和 mask 统计写入
`outputs/remote_control_mapping/runtime/status.json` 和 `events.jsonl`。

## 自主搜索

WebUI 的“高级选项”可在 `Grounded-SAM`、`YOLOE-Seg` 和 `VLM` 之间选择。
CLI 接受：

```bash
python scripts/go2w/run_semantic_exploration.py --detector yoloe_seg --help
```

YOLOE 的健康检查会在创建探索器、下发 Nav2 目标前完成。未完成现场清场、遥控
急停和值守条件时，只做 Web/API 接线测试或 dry-run，不授权运动。

## 回滚

单次回滚只需把入口切回 `grounded_sam`；无需卸载 YOLOE 环境，也不删除历史
会话。配置默认值仍然是 Grounded-SAM。
