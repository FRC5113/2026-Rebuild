"""Descriptions of the field and match state."""

import dataclasses
import typing

import robotpy_apriltag
import wpilib
from wpimath.geometry import (
    Pose2d,
    Pose3d,
    Rotation2d,
    Translation2d,
)

apriltag_layout = robotpy_apriltag.AprilTagFieldLayout.loadField(
    robotpy_apriltag.AprilTagField.k2026RebuiltWelded
)

TagId = typing.Literal[
    1,
    2,
    3,
    4,
    5,
    6,
    7,
    8,
    9,
    10,
    11,
    12,
    13,
    14,
    15,
    16,
    17,
    18,
    19,
    20,
    21,
    22,
    23,
    24,
    25,
    26,
    27,
    28,
    29,
    30,
    31,
    32,
]

get_fiducial_pose = typing.cast(
    typing.Callable[[TagId], Pose3d],
    apriltag_layout.getTagPose,
)

APRILTAGS = apriltag_layout.getTags()


@dataclasses.dataclass(slots=True)
class Tag2d:
    id: TagId
    pose: Pose2d


APRILTAGS_2D = [
    Tag2d(typing.cast(TagId, tag.ID), tag.pose.toPose2d()) for tag in APRILTAGS
]

FIELD_WIDTH = apriltag_layout.getFieldWidth()
FIELD_LENGTH = apriltag_layout.getFieldLength()

RED_HUB_POS = (
    get_fiducial_pose(4).translation().toTranslation2d()
    + get_fiducial_pose(10).translation().toTranslation2d()
) / 2

BLUE_HUB_POS = (
    get_fiducial_pose(20).translation().toTranslation2d()
    + get_fiducial_pose(26).translation().toTranslation2d()
) / 2


def get_hub_pos(is_red: bool):
    if is_red:
        return RED_HUB_POS
    else:
        return BLUE_HUB_POS


def field_flip_pose2d(p: Pose2d):
    return Pose2d(
        field_flip_translation2d(p.translation()),
        field_flip_rotation2d(p.rotation()),
    )


def field_flip_rotation2d(r: Rotation2d):
    return Rotation2d(-r.cos(), r.sin())


def field_flip_translation2d(t: Translation2d):
    return Translation2d(FIELD_LENGTH - t.x, t.y)


# This will default to the blue alliance if a proper link to the driver station has not yet been established
def is_red() -> bool:
    return wpilib.DriverStation.getAlliance() == wpilib.DriverStation.Alliance.kRed
