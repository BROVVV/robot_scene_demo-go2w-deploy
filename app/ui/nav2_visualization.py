"""Matplotlib visualization for ROS-independent Nav2 payloads."""
from __future__ import annotations

def render_nav2_path_figure(path: dict, current_pose: dict | None = None):
    from matplotlib.figure import Figure
    poses=path.get("poses", [])
    figure=Figure(figsize=(7,5)); axis=figure.subplots()
    if poses:
        axis.plot([p["x"] for p in poses],[p["y"] for p in poses],"-o",markersize=3,label="Planned Path")
        axis.scatter([poses[0]["x"]],[poses[0]["y"]],color="green",s=70,label="Start")
        axis.scatter([poses[-1]["x"]],[poses[-1]["y"]],color="red",s=100,marker="*",label="Goal")
    if current_pose:
        axis.scatter([current_pose["x"]],[current_pose["y"]],color="orange",s=70,label="Current Pose")
    axis.set_xlabel("X (m)"); axis.set_ylabel("Y (m)"); axis.set_aspect("equal",adjustable="datalim")
    axis.grid(True); axis.legend()
    return figure

def render_navigation_plan_figure(plan: dict):
    from matplotlib.figure import Figure
    poses=plan.get("path", [])
    waypoints=plan.get("waypoints", [])
    figure=Figure(figsize=(7,5)); axis=figure.subplots()
    if poses:
        axis.plot([p["x"] for p in poses],[p["y"] for p in poses],"-o",markersize=3,label="Visual Path")
        axis.scatter([poses[0]["x"]],[poses[0]["y"]],color="green",s=80,label="Start")
        axis.scatter([poses[-1]["x"]],[poses[-1]["y"]],color="red",s=110,marker="*",label="Goal")
    colors={"target":"red","candidate":"purple","observation":"orange","frontier":"blue","trajectory":"gray","start":"green"}
    for waypoint in waypoints:
        pose=waypoint.get("pose") or {}
        waypoint_type=waypoint.get("waypoint_type","trajectory")
        axis.scatter([pose.get("x",0)],[pose.get("y",0)],s=55,color=colors.get(waypoint_type,"gray"),label=waypoint_type)
    scale=plan.get("scale_status","relative")
    unit="m" if scale=="metric" else "relative units"
    axis.set_title("Visual Navigation Plan" if scale!="metric" else "Metric Visual Navigation Plan")
    axis.set_xlabel(f"X ({unit})"); axis.set_ylabel(f"Y ({unit})")
    axis.set_aspect("equal",adjustable="datalim")
    axis.grid(True)
    handles, labels = axis.get_legend_handles_labels()
    dedup = dict(zip(labels, handles))
    if dedup: axis.legend(dedup.values(), dedup.keys())
    return figure
