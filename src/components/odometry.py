from photonlibpy.photonPoseEstimator import PhotonPoseEstimator
from robotpy_apriltag import AprilTagFieldLayout
from wpilib import Field2d, SmartDashboard

from components.swerve_drive import SwerveDrive
from lemonlib.vision import LemonCamera


class Odometry:
    camera_front_left: LemonCamera
    # camera_front_right: LemonCamera
    # camera_back_left: LemonCamera
    # camera_back_right: LemonCamera
    field_layout: AprilTagFieldLayout
    swerve_drive: SwerveDrive
    estimated_field: Field2d

    def setup(self):
        self.camera_pose_estimator_front_left = PhotonPoseEstimator(
            self.field_layout,
            self.camera_front_left.camera_to_bot,
        )
        # self.camera_pose_estimator_front_right = PhotonPoseEstimator(
        #     self.field_layout,
        #     self.camera_front_right.camera_to_bot,
        # )
        # self.camera_pose_estimator_back_left = PhotonPoseEstimator(
        #     self.field_layout,
        #     self.camera_back_left.camera_to_bot,
        # )
        # self.camera_pose_estimator_back_right = PhotonPoseEstimator(
        #     self.field_layout,
        #     self.camera_back_right.camera_to_bot,
        # )

        SmartDashboard.putData("Estimated Field", self.estimated_field)

    def execute(self):
        self._process_latest_result(
            self.camera_front_left, self.camera_pose_estimator_front_left
        )
        # self._process_latest_result(
        #     self.camera_front_right, self.camera_pose_estimator_front_right
        # )
        # self._process_latest_result(
        #     self.camera_back_left, self.camera_pose_estimator_back_left
        # )
        # self._process_latest_result(
        #     self.camera_back_right, self.camera_pose_estimator_back_right
        # )
        self.estimated_field.setRobotPose(self.swerve_drive.get_estimated_pose())

    def _process_latest_result(
        self, camera: LemonCamera, estimator: PhotonPoseEstimator
    ):
        results = camera.getAllUnreadResults()
        if not results:
            return
        result = results[-1]
        camEstPose = estimator.estimateCoprocMultiTagPose(result)
        if camEstPose is None:
            return
        self.swerve_drive.addVisionPoseEstimate(
            camEstPose.estimatedPose, camEstPose.timestampSeconds
        )
