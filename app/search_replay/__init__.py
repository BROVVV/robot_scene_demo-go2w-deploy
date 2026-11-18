"""Search replay: read-only recording, offline rendering and Web playback.

规划书：docs/GO2W_自主搜索二维地图与轨迹回放视频_实施与验收计划_20261021.md

职责边界（刻意分开，互不混淆）：

* ``record_search_replay_ros.py``（系统 Python，独立进程）只订阅 /map、TF、
  定位状态与轮式里程，另开一个只读 HTTP 客户端取相机颜色帧；**不创建任何
  运动客户端、不发布 cmd_vel、不发导航目标、不启动第二套建图**。
* ``models`` / ``storage`` / ``timeline`` / ``trajectory`` 是纯数据模块，
  Web 进程和离线 CLI 共用，不 import rclpy。
* ``renderer`` / ``encoder`` 只读持久录制包，生成 MP4/PNG/JPEG；ffmpeg 用
  参数数组启动，不经过 shell。
* ``manager`` 负责把搜索会话生命周期接到采集与后台渲染任务上；搜索事件
  回调里只做有界入队，故障永远不改变搜索/停车/急停路径。
"""
