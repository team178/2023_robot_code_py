from commands2 import Command


class AutoSelector:
    
    def __init__(self, robot: "Robot"):
        self.robot = robot

    def get_selected(self) -> Command:
        pass