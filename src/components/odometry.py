from wpilib import Field2d, SmartDashboard, Timer
from wpimath.geometry import Transform3d, Pose2d
from photonlibpy.photonCamera import PhotonCamera
from photonlibpy.photonPoseEstimator import PhotonPoseEstimator
from robotpy_apriltag import AprilTagFieldLayout

from lemonlib.vision import LemonCamera

from components.swerve_drive import SwerveDrive


class Odometry:
    camera_front_left: LemonCamera
    camera_front_right: LemonCamera
    camera_back_left: LemonCamera
    camera_back_right: LemonCamera
    field_layout: AprilTagFieldLayout
    swerve_drive: SwerveDrive
    estimated_field: Field2d

    def setup(self):
        self.camera_pose_estimator_front_left = PhotonPoseEstimator(
            self.field_layout,
            self.camera_front_left.camera_to_bot,
        )
        self.camera_pose_estimator_front_right = PhotonPoseEstimator(
            self.field_layout,
            self.camera_front_right.camera_to_bot,
        )
        self.camera_pose_estimator_back_left = PhotonPoseEstimator(
            self.field_layout,
            self.camera_back_left.camera_to_bot,
        )
        self.camera_pose_estimator_back_right = PhotonPoseEstimator(
            self.field_layout,
            self.camera_back_right.camera_to_bot,
        )

        SmartDashboard.putData("Estimated Field", self.estimated_field)

    def execute(self):
        for result in self.camera_front_left.getAllUnreadResults():
            camEstPose = (
                self.camera_pose_estimator_front_left.estimateCoprocMultiTagPose(result)
            )
            if camEstPose is None:
                continue

            self.swerve_drive.addVisionPoseEstimate(
                camEstPose.estimatedPose, camEstPose.timestampSeconds
            )
        for result in self.camera_front_right.getAllUnreadResults():
            camEstPose = (
                self.camera_pose_estimator_front_right.estimateCoprocMultiTagPose(
                    result
                )
            )
            if camEstPose is None:
                continue

            self.swerve_drive.addVisionPoseEstimate(
                camEstPose.estimatedPose, camEstPose.timestampSeconds
            )
        for result in self.camera_back_left.getAllUnreadResults():
            camEstPose = (
                self.camera_pose_estimator_back_left.estimateCoprocMultiTagPose(result)
            )
            if camEstPose is None:
                continue

            self.swerve_drive.addVisionPoseEstimate(
                camEstPose.estimatedPose, camEstPose.timestampSeconds
            )
        for result in self.camera_back_right.getAllUnreadResults():
            camEstPose = (
                self.camera_pose_estimator_back_right.estimateCoprocMultiTagPose(result)
            )
            if camEstPose is None:
                continue

            self.swerve_drive.addVisionPoseEstimate(
                camEstPose.estimatedPose, camEstPose.timestampSeconds
            )
        self.estimated_field.setRobotPose(self.swerve_drive.get_estimated_pose())
