from autonomous.auto_base import AutoBase

# from lemonlib import LemonCamera


"""
Trajectories:
- climb
"""

"""
States:

"""


class state_test(AutoBase):
    MODE_NAME = "state-test"

    def __init__(self):
        super().__init__(
            [
                "climb",
            ]
        )
