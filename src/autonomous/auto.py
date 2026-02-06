from autonomous.auto_base import AutoBase
from magicbot import state, timedstate


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
  
class custom_state_test(AutoBase):
    MODE_NAME = "custom state test"

    def __init__(self):
        super().__init__([
            "state:custom_state"
        ])

    @state
    def custom_state(self):
        print("hi")
        self.next_state("next_step")

class add_step_test(AutoBase):
    MODE_NAME = "add step test"

    def __init__(self):
        super().__init__([
            AutoBase.add_trajectory("hub_shoot")
            AutoBase.add_action("shoot")
        ])

class combined_thing(AutoBase):
    MODE_NAME = "combined test"

    def __init__(self):
        super().__init__([
            AutoBase.add_action("custom_state")
        ])

    @state
    def custom_state(self):
        print("hi")
        self.next_state("next_step")

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
    @state
    def custom_state(self):
        print(hi)



class hub_outpost_shoot_climb(AutoBase):
    MODE_NAME = "Hub>Outpost>Shoot>Climb"

    def __init__(self):
        super().__init__(
            [
                "trajectory:hub_outpost",
                "state:outpost_wait",
                "trajectory:outpost_shoot",
                "state:shoot",
                "trajectory:shoot_climb",
                "state:climb",
            ]
        )


class hub_shoot(AutoBase):
    MODE_NAME = "Hub>Shoot"

    def __init__(self):
        super().__init__(
            [
                "trajectory:hub_shoot",
                "state:shoot",
            ]
        )


class hub_shoot_climb(AutoBase):
    MODE_NAME = "Hub>Shoot>Climb"

    def __init__(self):
        super().__init__(
            [
                "trajectory:hub_shoot",
                "state:shoot",
                "trajectory:shoot_climb",
                "state:climb",
            ]
        )
