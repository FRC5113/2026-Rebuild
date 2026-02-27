from autonomous.auto_base import AutoBase

"""
Trajectories: (start with trajectory:)
- hub_climb
- outpost_shoot
- hub_outpost
- shoot_climb
- hub_shoot
"""

"""
States: (start with state:)
- shoot
- climb
- outpost_wait
"""


class hub_outpost_shoot(AutoBase):
    MODE_NAME = "Hub>Outpost>Shoot"

    def __init__(self):
        super().__init__(
            [
                "trajectory:hub_outpost",
                "state:outpost_wait",
                "trajectory:outpost_shoot",
                "state:shoot",
            ]
        )


class hub_shoot(AutoBase):
    MODE_NAME = "Hub>Shoot"
    DEFAULT = True

    def __init__(self):
        super().__init__(
            [
                "trajectory:hub_shoot",
                "state:shoot",
            ]
        )
