import unittest
import math
from unittest.mock import MagicMock, patch
import cv2
import numpy as np

# Assuming PickPlaceRobot and necessary constants are in pick_place_sort.py
from pick_place_sort import PickPlaceRobot, PIXEL_TO_MM, GRIPPER_OPEN_ANGLE, GRIPPER_CLOSED_ANGLE  # Import the class

class TestPickPlaceRobot(unittest.TestCase):
    
    def setUp(self):
        self.robot = PickPlaceRobot("192.168.4.1")  # Initialize an instance of your class here
        # Define a sample image.  For testing, we don't need a real image.
        self.test_image = np.zeros((100, 100, 3), dtype=np.uint8) # Create a black image for testing

    def test_img_x_correction(self):
        self.assertAlmostEqual(self.robot.img_x_correction(10), 9.21, places=2)
        self.assertAlmostEqual(self.robot.img_x_correction(20), 19.42, places=2)

    def test_img_y_correction(self):
        self.assertAlmostEqual(self.robot.img_y_correction(10), 14.88, places=2)
        self.assertAlmostEqual(self.robot.img_y_correction(20), 24.48, places=2)
    
    def test_sort_cubes_no_cubes(self):
        """Test the scenario where no cubes are detected."""
        # Mock the detect_cubes_w_anchor method to return an empty list
        self.robot.detect_cubes_w_anchor = MagicMock(return_value=[])

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
        
    def test_sort_cubes_one_cube(self):
        """Test sorting with a single cube."""
        # Mock detect_cubes_w_anchor to return a single cube
        self.robot.detect_cubes_w_anchor = MagicMock(return_value=[(10, 20, -120, 'redcube')])  # Example cube data

        # Mock the pick_object and place_object methods to record their calls
        self.robot.pick_object = MagicMock()
        self.robot.place_object = MagicMock()

        # Call sort_cubes
        self.robot.sort_cubes()

        # Assertions:
        #  - Check that pick_object and place_object were called once.
        self.assertEqual(self.robot.pick_object.call_count, 1)
        self.assertEqual(self.robot.place_object.call_count, 1)

        #  -  Check the arguments passed to pick_object and place_object.  These
        #     should match the data from the mocked detect_cubes_w_anchor, and the
        #      color position.
        self.robot.pick_object.assert_called_with(10, 20, -120)  # Example coordinates
        self.robot.place_object.assert_called_with(8, -75, -120)  #redcube placement

    def test_sort_cubes_multiple_cubes(self):
        """Test sorting multiple cubes with different colors and distances."""
        # Mock detect_cubes_w_anchor to return multiple cubes
        self.robot.detect_cubes_w_anchor = MagicMock(return_value=[
            (10, 20, -120, 'redcube'),
            (30, 40, -120, 'bluecube'),
            (20, 30, -120, 'greencube'),
        ])

        # Mock pick_object and place_object
        self.robot.pick_object = MagicMock()
        self.robot.place_object = MagicMock()

        # Call sort_cubes
        self.robot.sort_cubes()

        # Assertions:
        #  - Check that pick_object and place_object were called three times each
        self.assertEqual(self.robot.pick_object.call_count, 3)
        self.assertEqual(self.robot.place_object.call_count, 3)

        # check call order.
        calls = [call[1] for call in self.robot.pick_object.mock_calls]
        expected_calls = [(10, 20, -120), (20, 30, -120), (30, 40, -120)]
        self.assertEqual(calls, expected_calls)

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
