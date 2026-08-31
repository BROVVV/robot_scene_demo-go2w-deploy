import unittest

from app.live_robot.navigation_gate import (
    EXECUTE_REQUIRED,
    NavigationReadiness,
    evaluate_navigation_gate,
    validate_execute_gate_payload,
)


class LiveNavigationGateTest(unittest.TestCase):
    def test_defaults_block_plan_and_execute(self):
        self.assertFalse(evaluate_navigation_gate("plan_only").allowed)
        self.assertFalse(evaluate_navigation_gate("execute").allowed)

    def test_plan_only_does_not_require_motion_chain(self):
        evidence = {name: True for name in evaluate_navigation_gate("plan_only").required_conditions}
        result = evaluate_navigation_gate("plan_only", evidence)
        self.assertTrue(result.allowed)
        self.assertFalse(result.evidence.lease_valid)

    def test_execute_requires_every_condition(self):
        evidence = {name: True for name in EXECUTE_REQUIRED}
        evidence["remote_override_clear"] = False
        result = evaluate_navigation_gate("execute", evidence)
        self.assertFalse(result.allowed)
        self.assertEqual(result.blocking_conditions, ("remote_override_clear",))

    def test_truthy_strings_cannot_open_a_gate(self):
        result = evaluate_navigation_gate("plan_only", {name: "true" for name in EXECUTE_REQUIRED})
        self.assertFalse(result.allowed)

    def test_serialized_execute_result_validates(self):
        result = evaluate_navigation_gate(
            "execute", NavigationReadiness(**{name: True for name in EXECUTE_REQUIRED})
        ).to_dict()
        validate_execute_gate_payload(result)
        result["blocking_conditions"] = ["operator_armed"]
        result["allowed"] = False
        with self.assertRaisesRegex(ValueError, "CAPABILITY_GATE_BLOCKED"):
            validate_execute_gate_payload(result)


if __name__ == "__main__":
    unittest.main()
