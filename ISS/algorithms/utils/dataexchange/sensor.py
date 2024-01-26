import numpy as np
from typing import Dict

from ISS.algorithms.utils.dataexchange.quaternion import ISSQuaternion

class ISSSensor:
    def __init__() -> None:
        pass

class ISSCamera(ISSSensor):
    def __init__() -> None:
        pass

    def update_camera_info_from_ros_msg(self, camera_info: Dict):
        self._height = camera_info["height"]
        self._width = camera_info["width"]
        # support models are listed in "sensor_msgs/distortion_models.h" in ros msg
        self._distortion_model = camera_info["distortion_model"]
        # distortion coefficients
        # The distortion parameters, size depending on the distortion model.
        # # For "plumb_bob", the 5 parameters are: (k1, k2, t1, t2, k3).
        # float64[] D

        # # Intrinsic camera matrix for the raw (distorted) images.
        # #     [fx  0 cx]
        # # K = [ 0 fy cy]
        # #     [ 0  0  1]
        # # Projects 3D points in the camera coordinate frame to 2D pixel
        # # coordinates using the focal lengths (fx, fy) and principal point
        # # (cx, cy).
        # float64[9]  K # 3x3 row-major matrix

        # # Rectification matrix (stereo cameras only)
        # # A rotation matrix aligning the camera coordinate system to the ideal
        # # stereo image plane so that epipolar lines in both stereo images are
        # # parallel.
        # float64[9]  R # 3x3 row-major matrix

        # # Projection/camera matrix
        # #     [fx'  0  cx' Tx]
        # # P = [ 0  fy' cy' Ty]
        # #     [ 0   0   1   0]
        # # By convention, this matrix specifies the intrinsic (camera) matrix
        # #  of the processed (rectified) image. That is, the left 3x3 portion
        # #  is the normal camera intrinsic matrix for the rectified image.
        # # It projects 3D points in the camera coordinate frame to 2D pixel
        # #  coordinates using the focal lengths (fx', fy') and principal point
        # #  (cx', cy') - these may differ from the values in K.
        # # For monocular cameras, Tx = Ty = 0. Normally, monocular cameras will
        # #  also have R = the identity and P[1:3,1:3] = K.
        # # For a stereo pair, the fourth column [Tx Ty 0]' is related to the
        # #  position of the optical center of the second camera in the first
        # #  camera's frame. We assume Tz = 0 so both cameras are in the same
        # #  stereo image plane. The first camera always has Tx = Ty = 0. For
        # #  the right (second) camera of a horizontal stereo pair, Ty = 0 and
        # #  Tx = -fx' * B, where B is the baseline between the cameras.
        # # Given a 3D point [X Y Z]', the projection (x, y) of the point onto
        # #  the rectified image is given by:
        # #  [u v w]' = P * [X Y Z 1]'
        # #         x = u / w
        # #         y = v / w
        # #  This holds for both images of a stereo pair.
        # float64[12] P # 3x4 row-major matrix
        self._D = np.array(camera_info["D"])
        self._K = np.array(camera_info["K"]).reshape(3, 3)
        self._R = np.array(camera_info["R"]).reshape(3, 3)
        self._P = np.array(camera_info["P"]).reshape(3, 4)
        # operational parameters
        # check if binning_x, binning_y, roi exist in camera_info_msg
        if "binning_x" in camera_info:
            self._binning_x = camera_info["binning_x"]
        if "binning_y" in camera_info:
            self._binning_y = camera_info["binning_y"]
        if "roi" in camera_info:
            self._roi = camera_info["roi"]

    def update_image_from_ros_msg(self, image: Dict):
        
        if hasattr(image, "format"):
            # CompressedImage
            self._format = image["format"]
            self._data = np.array(image["data"])
        else:
            # Normal Image
            self._height = image["height"]
            self._width = image["width"]
            self._encoding = image["encoding"]
            self._is_bigendian = image["is_bigendian"]
            # step: full row length in bytes, commonly width * 3 or width * 4
            self._step = image["step"]
            # data: step * rows
            self._data = np.array(image["data"])

    def get_image_raw_data(self):
        return self._data.copy()
    
    def is_empty(self):
        return self._data is None

class ISSLidar(ISSSensor):
    def __init__() -> None:
        pass

class ISSIMU(ISSSensor):
    def __init__() -> None:
        pass

    def read_from_ros_msg(self, imu_msg):
        # orientation
        self._orientation = ISSQuaternion(imu_msg.orientation)
        # orientation covariance
        self._orientation_cov = np.array(imu_msg.orientation_covariance).reshape(3, 3)
        # angular velocity
        self._angular_velocity = np.array(imu_msg.angular_velocity)
        # angular velocity covariance
        self._angular_velocity_cov = np.array(imu_msg.angular_velocity_covariance).reshape(3, 3)
        # linear acceleration
        self._linear_acceleration = np.array(imu_msg.linear_acceleration)
        # linear acceleration covariance
        self._linear_acceleration_cov = np.array(imu_msg.linear_acceleration_covariance).reshape(3, 3)

