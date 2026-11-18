"""``create_nav2_search_backend`` 的 TF 位姿缝（计划书 §6.1 / §3.2）。

这条缝原先没有任何测试：``tests/test_nav2_search_backend.py`` 全部绕过工厂、直接
构造 ``Nav2SearchBackend(pose_provider=...)``，于是工厂闭包里那个把 ``timeout``
按 float 传给 tf2 的写法一直没被发现 —— tf2 的签名是 ``timeout=Duration()``，
拿到 float 时在 ``timeout != Duration()`` 这一句就抛 ``TypeError``，而
``_pose_provider`` 的 ``except Exception`` 把它当成「TF 没到」，静默返回 ``None``。

后果不是少一条日志：``AutonomousExplorer`` 会拿 ``backend.get_pose()`` **覆盖**
``observation.pose``（``autonomous_explorer.py:644``），位姿一空，§6.7 的
``NAVIGATE_POSE`` 就表达不出来，候选生成器只报一句 ``local_executor_no_primitive``。

这里的假 tf2 复刻实测到的两条真实语义，否则测试没有分辨力：
  1. ``timeout`` 不是 ``Duration`` → ``TypeError``；
  2. 按 node 时钟的 ``now()`` 查 → 比最新数据晚约 37 ms → 外推失败；
     查「最新一帧」（``Time()``）才成功。
对照组 ``test_fake_tf_reproduces_real_tf2_semantics`` 就是证明这两条真的会触发。
"""

from __future__ import annotations

import sys
import types
from contextlib import contextmanager

import pytest

from app.navigation import nav2_search_backend as nb
from app.navigation.robot_backend import PoseQuality

# ---------------------------------------------------------------------------
# 假 tf2 / rclpy：语义照抄实测行为，不做「宽容化」
# ---------------------------------------------------------------------------


class ExtrapolationError(Exception):
    """tf2_ros 的外推异常，名字与真实类一致以便测试读起来不别扭。"""


class FakeTime:
    def __init__(self, *, kind: str, nanoseconds: int = 0) -> None:
        self.kind = kind  # "latest" | "now"
        self.nanoseconds = nanoseconds


class FakeDuration:
    def __init__(self, *, seconds: float = 0.0, nanoseconds: int = 0) -> None:
        self.seconds = float(seconds)
        self.nanoseconds = int(nanoseconds)


class _Translation:
    def __init__(self, x: float, y: float) -> None:
        self.x = x
        self.y = y


class _Rotation:
    def __init__(self, x: float, y: float, z: float, w: float) -> None:
        self.x, self.y, self.z, self.w = x, y, z, w


class _TransformStamped:
    def __init__(self, x: float, y: float) -> None:
        self.header = types.SimpleNamespace(stamp=types.SimpleNamespace(sec=1_791_439_845, nanosec=279_000_000))
        self.transform = types.SimpleNamespace(
            translation=_Translation(x, y), rotation=_Rotation(0.0, 0.0, 0.0, 1.0)
        )


class FakeBuffer:
    """复刻实测到的 tf2 Buffer 语义（见模块 docstring）。"""

    def __init__(self, *, x: float = -0.288, y: float = 0.018) -> None:
        self.x, self.y = x, y
        self.calls: list[dict[str, object]] = []

    def lookup_transform(self, target, source, time, timeout=None):
        self.calls.append(
            {"target": target, "source": source, "time": time, "timeout": timeout}
        )
        # 真实 tf2：timeout 只接受 Duration，float 在 `timeout != Duration()` 处抛错。
        if timeout is not None and not isinstance(timeout, FakeDuration):
            raise TypeError(
                "Can't compare duration with object of type: ", type(timeout)
            )
        # 真实 tf2：now() 比最新数据晚几十毫秒，直接查必然外推失败。
        if not isinstance(time, FakeTime) or time.kind != "latest":
            raise ExtrapolationError(
                "Lookup would require extrapolation into the future."
            )
        return _TransformStamped(self.x, self.y)


class FakeClock:
    def now(self) -> FakeTime:
        return FakeTime(kind="now", nanoseconds=1_791_439_845_316_670_000)


class FakePublisher:
    def publish(self, *args: object) -> None:
        pass


class FakeServiceClient:
    def service_is_ready(self) -> bool:
        return False

    def call_async(self, request: object) -> None:
        pass


class FakeNode:
    def __init__(self) -> None:
        self._clock = FakeClock()

    def create_subscription(self, *args: object, **kwargs: object) -> None:
        return None

    def create_publisher(self, *args: object, **kwargs: object) -> FakePublisher:
        return FakePublisher()

    def create_client(self, *args: object, **kwargs: object) -> FakeServiceClient:
        return FakeServiceClient()

    def get_clock(self) -> FakeClock:
        return self._clock


class FakeActionPort:
    def server_ready(self) -> bool:
        return True


@contextmanager
def _fake_ros():
    """只在调用工厂期间替换 ROS 模块；退出时逐个还原（不吃掉别人的替身）。

    父模块要做成真「包」（``__path__`` 非空），否则 ``from rcl_interfaces.srv
    import ...`` 会以 ``'rcl_interfaces' is not a package`` 直接失败。
    """
    def package(name: str) -> types.ModuleType:
        module = types.ModuleType(name)
        module.__path__ = []
        return module

    def leaf(name: str, **attrs: object) -> types.ModuleType:
        module = types.ModuleType(name)
        for key, value in attrs.items():
            setattr(module, key, value)
        return module

    rclpy = package("rclpy")
    qos = leaf(
        "rclpy.qos",
        DurabilityPolicy=types.SimpleNamespace(TRANSIENT_LOCAL=1, VOLATILE=2),
        ReliabilityPolicy=types.SimpleNamespace(RELIABLE=1, BEST_EFFORT=2),
        QoSProfile=lambda **kwargs: kwargs,
        qos_profile_sensor_data=object(),
    )
    duration = leaf("rclpy.duration", Duration=FakeDuration)
    rcltime = leaf("rclpy.time", Time=lambda *a, **k: FakeTime(kind="latest"))
    rclpy.qos, rclpy.duration, rclpy.time = qos, duration, rcltime

    geometry_msgs = package("geometry_msgs")
    geometry_msgs_msg = leaf("geometry_msgs.msg", Twist=object)
    geometry_msgs.msg = geometry_msgs_msg

    nav_msgs = package("nav_msgs")
    nav_msgs_msg = leaf("nav_msgs.msg", OccupancyGrid=object, Odometry=object)
    nav_msgs.msg = nav_msgs_msg

    std_msgs = package("std_msgs")
    std_msgs_msg = leaf("std_msgs.msg", Bool=object, String=object)
    std_msgs.msg = std_msgs_msg

    rcl_interfaces = package("rcl_interfaces")
    rcl_interfaces_msg = leaf(
        "rcl_interfaces.msg",
        Parameter=object,
        ParameterType=types.SimpleNamespace(PARAMETER_DOUBLE=2),
        ParameterValue=object,
    )
    rcl_interfaces_srv = leaf("rcl_interfaces.srv", SetParameters=object)
    rcl_interfaces.msg, rcl_interfaces.srv = rcl_interfaces_msg, rcl_interfaces_srv

    tf2_ros = leaf("tf2_ros", Buffer=FakeBuffer,
                   TransformListener=lambda *a, **k: None)

    injected = {
        "tf2_ros": tf2_ros,
        "rclpy": rclpy,
        "rclpy.qos": qos,
        "rclpy.duration": duration,
        "rclpy.time": rcltime,
        "geometry_msgs": geometry_msgs,
        "geometry_msgs.msg": geometry_msgs_msg,
        "nav_msgs": nav_msgs,
        "nav_msgs.msg": nav_msgs_msg,
        "std_msgs": std_msgs,
        "std_msgs.msg": std_msgs_msg,
        "rcl_interfaces": rcl_interfaces,
        "rcl_interfaces.msg": rcl_interfaces_msg,
        "rcl_interfaces.srv": rcl_interfaces_srv,
    }
    missing = object()
    saved = {name: sys.modules.get(name, missing) for name in injected}
    saved_clients = nb.create_nav2_action_clients
    sys.modules.update(injected)
    nb.create_nav2_action_clients = lambda node: (
        FakeActionPort(), FakeActionPort(), FakeActionPort()
    )
    try:
        yield
    finally:
        nb.create_nav2_action_clients = saved_clients
        for name, value in saved.items():
            if value is missing:
                sys.modules.pop(name, None)
            else:
                sys.modules[name] = value


def _factory(buffer: FakeBuffer):
    with _fake_ros():
        return nb.create_nav2_search_backend(FakeNode(), tf_buffer=buffer)


# ---------------------------------------------------------------------------
# 分辨力校准：先证明假 tf2 真的会因那两条语义而失败
# ---------------------------------------------------------------------------


def test_fake_tf_reproduces_real_tf2_semantics() -> None:
    buffer = FakeBuffer()
    with pytest.raises(TypeError):
        buffer.lookup_transform("map", "base_link", FakeTime(kind="latest"),
                                timeout=0.2)
    with pytest.raises(ExtrapolationError):
        buffer.lookup_transform("map", "base_link",
                                FakeTime(kind="now", nanoseconds=1),
                                timeout=FakeDuration(seconds=0.2))
    assert buffer.lookup_transform("map", "base_link", FakeTime(kind="latest"),
                                   timeout=FakeDuration(seconds=0.2)) is not None


# ---------------------------------------------------------------------------
# 被修的缝
# ---------------------------------------------------------------------------


def test_get_pose_returns_metric_pose_from_tf() -> None:
    backend = _factory(FakeBuffer(x=-0.288, y=0.018))
    pose = backend.get_pose()
    assert pose is not None, "工厂闭包里的 TF 查询必须真的查一次，不能静默返回 None"
    assert pose.frame_id == "map"
    assert pose.x == pytest.approx(-0.288)
    assert pose.y == pytest.approx(0.018)
    assert pose.quality is PoseQuality.METRIC
    assert pose.source == "tf:map->base_link"


def test_pose_lookup_uses_latest_and_a_real_duration() -> None:
    """这条把两处写法都钉住：查最新一帧、timeout 是 Duration。"""
    buffer = FakeBuffer()
    backend = _factory(buffer)
    assert backend.get_pose() is not None
    call = buffer.calls[-1]
    assert isinstance(call["time"], FakeTime) and call["time"].kind == "latest"
    assert isinstance(call["timeout"], FakeDuration)


def test_missing_tf_is_still_reported_as_no_pose() -> None:
    """对照组：真拿不到 TF 时仍然返回 None，不能为了「不空」编一个位姿。"""

    class AlwaysFailing(FakeBuffer):
        def lookup_transform(self, target, source, time, timeout=None):
            raise ExtrapolationError("no transform")

    assert _factory(AlwaysFailing()).get_pose() is None


def test_old_tf_cannot_be_presented_as_a_current_metric_pose():
    class OldBuffer(FakeBuffer):
        def lookup_transform(self, *args, **kwargs):
            result = super().lookup_transform(*args, **kwargs)
            result.header.stamp.sec -= 2
            return result
    assert _factory(OldBuffer()).get_pose() is None
