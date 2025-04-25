import unittest
import math
from unittest.mock import MagicMock, patch
import cv2
import numpy as np

# Assuming PickPlaceRobot and necessary constants are in pick_place_sort.py
from pick_place_sort import PickPlaceRobot, PIXEL_TO_MM, GRIPPER_OPEN_ANGLE, GRIPPER_CLOSED_ANGLE, cubes_labels  # Import the class

class TestPickPlaceRobot(unittest.TestCase):
    
    def setUp(self):
        self.robot = PickPlaceRobot("192.168.4.1")  # Initialize an instance of your class here
        # Define a sample image.  For testing, we don't need a real image.
        self.test_image = np.zeros((1920, 1080, 3), dtype=np.uint8) # Important: Use the correct image size

    def test_sort_cubes_no_cubes(self):
        """Test the scenario where no cubes are detected."""
        # Mock the detect_cubes_w_anchor method to return an empty list
        self.robot.detect_cubes = MagicMock(return_value=[])
        self.robot.capture_image = MagicMock(return_value=self.test_image)

        # Call sort_cubes
        self.robot.sort_cubes()

        # Assertions:
        #  -  If no cubes, the function should exit without errors.  We check that
        #     the pick/place methods were NOT called.
        self.robot.pick_object = MagicMock()
        self.robot.place_object = MagicMock()
        self.robot.sort_cubes()
        self.robot.pick_object.assert_not_called()
        self.robot.place_object.assert_not_called()

    def test_sort_cubes_one_cube_within_boundary(self):
        """Test sorting with a single cube within the 13mm boundary."""
        # Mock detect_cubes to return a single cube near the anchor
        self.robot.detect_cubes = MagicMock(return_value=[(8, -80, -120, 'redcube')]) #within 13mm of 8,-85
        self.robot.capture_image = MagicMock(return_value=self.test_image)

        # Mock the pick_object and place_object methods to record their calls
        self.robot.pick_object = MagicMock()
        self.robot.place_object = MagicMock()

        # Call sort_cubes
        self.robot.sort_cubes()

        # Assertions:
        #  - Check that pick_object and place_object were called once.
        self.assertEqual(self.robot.pick_object.call_count, 1)
        self.assertEqual(self.robot.place_object.call_count, 1)

        #  -  Check the arguments passed to pick_object and place_object.
        self.robot.pick_object.assert_called_with(8, -80, -120)
        self.robot.place_object.assert_called_with(8 + 25, -85, -120)  #redcube color position

    def test_sort_cubes_one_cube_outside_boundary(self):
        """Test sorting with a single cube outside the 13mm boundary."""
        # Mock detect_cubes to return a single cube far from the anchor
        self.robot.detect_cubes = MagicMock(return_value=[(30, -100, -120, 'redcube')])
        self.robot.capture_image = MagicMock(return_value=self.test_image)

        # Mock pick/place
        self.robot.pick_object = MagicMock()
        self.robot.place_object = MagicMock()

        # Call sort_cubes
        self.robot.sort_cubes()
        
        self.assertEqual(self.robot.pick_object.call_count, 1)
        self.assertEqual(self.robot.place_object.call_count, 1)
        self.robot.pick_object.assert_called_with(30, -100, -120)
        self.robot.place_object.assert_called_with(8 + 25, -85, -120)  # Red cube placement

    def test_sort_cubes_multiple_cubes(self):
        """Test sorting multiple cubes with different colors and distances."""
        # Mock detect_cubes to return multiple cubes
        self.robot.detect_cubes = MagicMock(return_value=[
            (8, -80, -120, 'redcube'),  # Within boundary
            (30, -100, -120, 'bluecube'), # Outside boundary
            (10, -90, -120, 'greencube'), #within
        ])
        self.robot.capture_image = MagicMock(return_value=self.test_image)

        # Mock pick_object and place_object
        self.robot.pick_object = MagicMock()
        self.robot.place_object = MagicMock()

        # Call sort_cubes
        self.robot.sort_cubes()

        # Assertions:
        #  - Check that pick_object and place_object were called three times each
        self.assertEqual(self.robot.pick_object.call_count, 3)
        self.assertEqual(self.robot.place_object.call_count, 3)

        #  -  Check the order of calls and arguments.  The cubes should be processed
        #     in the order they are returned by detect_cubes (which is arbitrary in this test).
        expected_pick_calls = [
            ((8, -80, -120),),
            ((30, -100, -120),),
            ((10, -90, -120),),
        ]
        expected_place_calls = [
            ((8 + 25, -85, -120),),  # Red
            ((8, -85 + 25, -120),),  # Blue
            ((8, -85 + 50, -120),),  # Green
        ]
        self.robot.pick_object.assert_has_calls(
            [call(*args) for args in expected_pick_calls], any_order=False
        )
        self.robot.place_object.assert_has_calls(
            [call(*args) for args in expected_place_calls], any_order=False
        )
    
    def test_connect_arm_success(self):
        """Test successful connection to the robot arm's Wi-Fi."""
        # Patch subprocess.Popen to simulate a successful connection
        mock_popen = MagicMock()
        mock_process = MagicMock()
        mock_process.returncode = 0  # Simulate success
        mock_process.communicate.return_value = (b"success", b"")  # Simulate no error message
        mock_popen.return_value = mock_process

        with patch('subprocess.Popen', mock_popen):
            result = self.robot.connect_arm()
            self.assertTrue(result)

    def test_connect_arm_failure(self):
        """Test failed connection to the robot arm's Wi-Fi."""
        # Patch subprocess.Popen to simulate a failed connection
        mock_popen = MagicMock()
        mock_process = MagicMock()
        mock_process.returncode = 1  # Simulate failure
        mock_process.communicate.return_value = (b"", b"error message")  # Simulate an error message
        mock_popen.return_value = mock_process

        with patch('subprocess.Popen', mock_popen):
            result = self.robot.connect_arm()
            self.assertFalse(result)

    def test_capture_image_success(self):
        """Test successful image capture."""
        # Patch cv2.VideoCapture and cv2.imread
        mock_video_capture = MagicMock()
        mock_video_capture.return_value.read.return_value = (True, self.test_image)
        mock_imwrite = MagicMock()

        with patch('cv2.VideoCapture', mock_video_capture), patch('cv2.imwrite', mock_imwrite), patch('cv2.imshow'), patch('cv2.waitKey'), patch('cv2.destroyWindow'):
            image = self.robot.capture_image()
            self.assertTrue(np.array_equal(image, self.test_image))

    def test_capture_image_failure(self):
        """Test failed image capture."""
        # Patch cv2.VideoCapture to simulate a failure
        mock_video_capture = MagicMock()
        mock_video_capture.return_value.read.return_value = (False, None)

        with patch('cv2.VideoCapture', mock_video_capture):
            with self.assertRaises(Exception) as context:
                self.robot.capture_image()
            self.assertEqual(str(context.exception), "No image detected. Please! try again")

if __name__ == '__main__':
    unittest.main()
