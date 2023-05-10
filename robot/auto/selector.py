from wpilib import SendableChooser

class AutoSelector(SendableChooser):

    _instance: "AutoSelector" = None
    robot: "Robot" = None

    def __new__(cls, *args, **kwargs):
        # Making this class a singleton, so there's ever only one instance
        if cls._instance is None:
            cls._instance = super(AutoSelector, cls).__new__(cls, *args, **kwargs)
        return cls._instance

    def __init__(self, robot: "Robot"):
        super().__init__() # don't forget this or it segfaults?
        self.robot = robot
        self.setDefaultOption("None", None)

        # this is NOT good practice but it'll work
        import robot.auto.routes
    
    @classmethod
    def route(cls, name: str):
        """
        Utility decorator to add a command to the AutoSelector
        """
        def _dec(func):
            cls._instance.addOption(name, func(cls._instance.robot))
            return func
        return _dec