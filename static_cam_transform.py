import cv2

class StaticCamTransform:
    def __init__(self, camera_matrix, dist_coeffs, px_to_robox, px_to_roboy):
        """
        Initialize the StaticCamTransform with camera matrix, distortion coefficients,
        and conversion factors.

        :param camera_matrix: Camera intrinsic matrix.
        :param dist_coeffs: Distortion coefficients.
        :param px_to_robox: pixels to robot x-axis conversion factor.
        :param px_to_roboy: pixels to robot y-axis conversion factor.
        """
        self.camera_matrix = camera_matrix
        self.dist_coeffs = dist_coeffs
        self.px_to_robox = px_to_robox
        self.px_to_roboy = px_to_roboy

    def undistort(self, image):
        """
        Undistort an image using the camera matrix and distortion coefficients.

        :param image: Input image to be undistorted.
        :return: Undistorted image.
        """
        # Assuming OpenCV is used for image processing
        return cv2.undistort(image, self.camera_matrix, self.dist_coeffs)

    def transform_px_to_roboxy(self, obj1_x_mid, obj1_y_mid, obj2_x_mid, obj2_y_mid):
        """
        Transform pixel coordinates to robot coordinates.
        Use undistorted coordinates for the transformation.

        :param obj1_x_mid: X coordinate of the first object in pixels.
        :param obj1_y_mid: Y coordinate of the first object in pixels.
        :param obj2_x_mid: X coordinate of the second object in pixels.
        :param obj2_y_mid: Y coordinate of the second object in pixels.
        :return: Tuple containing transformed distances between objects
        """
        delta_x = (obj1_x_mid - obj2_x_mid) * self.px_to_robox
        delta_y = (obj1_y_mid - obj2_y_mid) * self.px_to_roboy

        return delta_x, delta_y