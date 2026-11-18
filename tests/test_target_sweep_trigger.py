"""§44.7 / §44.8：VLM 目标扫描兜底的触发条件。"""

from __future__ import annotations

import unittest

from app.live_robot.target_sweep import decide_target_sweep

THRESHOLD = 4
INTERVAL = 10.0


def _step(
    *, has_target_like: bool = False, empty_cycles: int, last_sweep_time: float, now: float
):
    return decide_target_sweep(
        has_target_like=has_target_like,
        empty_cycles=empty_cycles,
        last_sweep_time=last_sweep_time,
        now=now,
        empty_cycles_threshold=THRESHOLD,
        interval_seconds=INTERVAL,
    )


class TargetSweepTriggerTest(unittest.TestCase):
    def test_does_not_sweep_before_the_empty_cycle_threshold(self) -> None:
        cycles = 0
        for index in range(THRESHOLD - 1):
            run, cycles, _ = _step(empty_cycles=cycles, last_sweep_time=0.0, now=100.0)
            self.assertFalse(run, f"第 {index + 1} 次空手就不该扫")

        self.assertEqual(cycles, THRESHOLD - 1)

    def test_sweeps_on_the_threshold_cycle(self) -> None:
        run, cycles, last = _step(
            empty_cycles=THRESHOLD - 1, last_sweep_time=0.0, now=100.0
        )

        self.assertTrue(run)
        # §44.7：扫完必须清零，否则下一周期会马上再扫一次。
        self.assertEqual(cycles, 0)
        self.assertEqual(last, 100.0)

    def test_next_cycle_does_not_sweep_immediately(self) -> None:
        # 刚扫完（last_sweep_time=100）又连续空手 4 次，但只过了 1 秒。
        cycles = 0
        for _ in range(THRESHOLD):
            run, cycles, last = _step(
                empty_cycles=cycles, last_sweep_time=100.0, now=101.0
            )
            self.assertFalse(run)

        self.assertEqual(cycles, THRESHOLD)
        self.assertEqual(last, 100.0)

    def test_sweeps_again_once_the_interval_elapsed(self) -> None:
        cycles = THRESHOLD - 1

        run, _, last = _step(
            empty_cycles=cycles, last_sweep_time=100.0, now=100.0 + INTERVAL
        )

        self.assertTrue(run)
        self.assertEqual(last, 100.0 + INTERVAL)

    def test_sixty_seconds_of_empty_cycles_sweeps_at_most_once_per_interval(self) -> None:
        # §64 效率：60 秒内 full-frame VLM sweep 不超过 interval 所允许的次数。
        # 每 0.5 秒一个检测周期（Grounded-SAM 约 2 Hz），全程没有 target-like 候选。
        # 起始 now 取一个真实的 monotonic 量级（节点刚起来时 last_sweep_time=0.0，
        # 所以第一次扫描只受空手周期数约束，不受 interval 约束）。
        start = 100000.0
        cycles = 0
        last = 0.0
        sweeps: list[float] = []
        for step in range(120):
            now = start + 0.5 * (step + 1)
            run, cycles, last = _step(empty_cycles=cycles, last_sweep_time=last, now=now)
            if run:
                sweeps.append(now)

        # 第一次扫描最早也要等 4 个空手周期（2 秒），之后每次都要再等满 interval。
        self.assertEqual(sweeps[0], start + 2.0)
        self.assertLessEqual(len(sweeps), 60.0 / INTERVAL + 1)
        for earlier, later in zip(sweeps, sweeps[1:]):
            self.assertGreaterEqual(later - earlier, INTERVAL)

    def test_target_like_candidate_resets_the_counter(self) -> None:
        # §44.8：只要有 target-like 候选，空手计数立刻清零，且不扫。
        run, cycles, last = _step(
            has_target_like=True, empty_cycles=THRESHOLD - 1, last_sweep_time=0.0,
            now=100.0,
        )

        self.assertFalse(run)
        self.assertEqual(cycles, 0)
        self.assertEqual(last, 0.0)


if __name__ == "__main__":
    unittest.main()
