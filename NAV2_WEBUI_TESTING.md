# Nav2 Web UI 验收

```bash
NAV2_ENABLED=true NAV2_MODE=offline_preview streamlit run streamlit_app.py
```

1. 旧分析功能在 Navigation2 关闭时正常。
2. 选择“离线路径预览”，输入目标并生成；页面出现黄色“非真实 Nav2”警告。
3. 检查总览、等比例 XY 路径、路径点、步骤警告、反馈、原始 JSON 与下载产物。
4. `plan_only` 不显示真实 `/cmd_vel`。
5. `execute` 四项确认缺一时按钮禁用；环境三项变量缺少时请求仍会被模型层拒绝。
6. 未安装 ROS 时 `plan_only` 显示 `NAV2_ROS_SETUP_NOT_FOUND`，不会出现模拟路径。
7. 执行中点击取消会创建 `cancel.request`；Worker 转为 canceling/canceled。
8. 日志不得展示 `.env` 全文或 API key。
