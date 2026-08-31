import unittest

from app.navigation.priority import priority_to_confidence


class NavigationPriorityTest(unittest.TestCase):
    def test_string_priority_maps_to_confidence(self):
        self.assertEqual(priority_to_confidence("high"), 0.8)
        self.assertEqual(priority_to_confidence("medium"), 0.55)
        self.assertEqual(priority_to_confidence("low"), 0.3)


if __name__ == "__main__":
    unittest.main()
