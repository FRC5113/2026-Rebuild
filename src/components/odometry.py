from wpilib import Field2d, SmartDashboard, Timer
from wpimath.geometry import Transform3d, Pose2d
from photonlibpy.photonCamera import PhotonCamera
from photonlibpy.photonPoseEstimator import PhotonPoseEstimator
from robotpy_apriltag import AprilTagFieldLayout

from lemonlib.vision import LemonCamera

from components.swerve_drive import SwerveDrive


class Odometry:
    camera_front: LemonCamera
    robot_to_camera_front: Transform3d
    field_layout: AprilTagFieldLayout
    swerve_drive: SwerveDrive
    estimated_field: Field2d

    def setup(self):
        self.camera_pose_estimator_front = PhotonPoseEstimator(
            self.field_layout,
            self.robot_to_camera_front,
        )

        SmartDashboard.putData("Estimated Field", self.estimated_field)

    def execute(self):
        for result in self.camera_front.getAllUnreadResults():
            camEstPose = self.camera_pose_estimator_front.estimateCoprocMultiTagPose(
                result
            )
            if camEstPose is None:
                camEstPose = (
                    self.camera_pose_estimator_front.estimateLowestAmbiguityPose(result)
                )

            self.estimated_field.setRobotPose(camEstPose.estimatedPose)
            self.swerve_drive.addVisionPoseEstimate(
                camEstPose.estimatedPose, camEstPose.timestampSeconds
            )
