import unittest
from sortlogic import packet_hash,color_sort

class TestPacketHash(unittest.TestCase):
    def test_packet_hash(self):
        # Mock input packets with RGB and XY values
        packets = [
            {"R": 255, "G": 0, "B": 0, "X": 0, "Y": 0},  # Red
            {"R": 0, "G": 255, "B": 0, "X": 0, "Y": 0},  # Green
            {"R": 0, "G": 0, "B": 255, "X": 0, "Y": 0},  # Blue
            {"R": 255, "G": 165, "B": 0, "X": 0, "Y": 0},  # Orange
            {"R": 255, "G": 255, "B": 0, "X": 0, "Y": 0},  # Yellow
            {"R": 128, "G": 128, "B": 128, "X": 0, "Y": 0},  # Unrecognized
        ]

        # Expected output
        expected_hash = {
            "Red": [(255, 0, 0)],
            "Red2": [],
            "Green": [(0, 255, 0)],
            "Blue": [(0, 0, 255)],
            "Yellow": [(255, 255, 0)],
            "Orange": [(255, 165, 0)],
        }

        # Run the function
        result = packet_hash(packets)
        # Assert the result matches the expected hash
        for color, values in expected_hash.items():  # Use .items() to get key-value pairs
            print(f"Testing color: {color}")
            for rgb in values:  # Iterate over the list of RGB tuples
                print(f'R: {rgb[0]} G: {rgb[1]} B: {rgb[2]}')  # Access tuple elements
            if expected_hash[color]:
                self.assertEqual(result[color], expected_hash[color])
            else:
                self.assertEqual(result[color], [])


class TestColorSort(unittest.TestCase):
    def test_color_sort(self):
        # Mock input: hashed_packet with packets grouped by color
        hashed_packet = {
            "Red": [{"R": 255, "G": 0, "B": 0}, {"R": 254, "G": 1, "B": 1}],
            "Orange": [{"R": 255, "G": 165, "B": 0}],
            "Yellow": [{"R": 255, "G": 255, "B": 0}],
            "Green": [{"R": 0, "G": 255, "B": 0}],
            "Blue": [{"R": 0, "G": 0, "B": 255}],
        }

        # Expected output: hashed_packet with assigned X and Y coordinates
        expected_output = {
            "Red": [
                {"R": 255, "G": 0, "B": 0, "X": -200, "Y": 600},
                {"R": 254, "G": 1, "B": 1, "X": -160, "Y": 600},
            ],
            "Orange": [{"R": 255, "G": 165, "B": 0, "X": -200, "Y": 550}],
            "Yellow": [{"R": 255, "G": 255, "B": 0, "X": -200, "Y": 500}],
            "Green": [{"R": 0, "G": 255, "B": 0, "X": -200, "Y": 450}],
            "Blue": [{"R": 0, "G": 0, "B": 255, "X": -200, "Y": 400}],
        }

        # Run the function
        result = color_sort(hashed_packet)
        print(f"Result: {result}")
        print(f"Expected: {expected_output}")
        # Assert the result matches the expected output
        self.assertEqual(result, expected_output)

        # Check if the coordinates are assigned correctly

if __name__ == "__main__":
    unittest.main()