from wpilib import SendableChooser


class AutoSelector(SendableChooser):
    """
    Auto selector class. Extends a SendableChooser to be a singleton so we
    don't have to worry about instances, and provides a decorator for easily
    adding auto commands.
    """

    _instance: "AutoSelector" = None
    robot: "Robot" = None

    _options = set()

    def __new__(cls, *args, **kwargs):
        # * Making this class a singleton, so there's ever only one instance
        if cls._instance is None:
            cls._instance = super(AutoSelector, cls).__new__(cls, *args, **kwargs)

        return cls._instance

    def __init__(self, robot: "Robot"):
        super().__init__()  #! don't forget this or it segfaults?

        self.robot = robot
        self.setDefaultOption("None", None)

        # ? this is NOT good practice but I need to import the routes
        # ? AFTER the robot instance has been passed to this class
        # ? so the decorator works. A little jank but whatever.
        import robot.auto.routes

        # TODO: Maybe use importlib instead?

    @classmethod
    def get_selected(cls):
        return cls._instance.getSelected()

    @classmethod
    def route(cls, name: str):
        """
        Utility decorator to add a command to the AutoSelector. Always passes in
        the current robot instance as the first argument.

        ## Example:
        ```
        from robot.auto.selector import AutoSelector as auto
        from commands2 import Command
        from commands2.cmd import print

        @auto.route("MyCommand")
        def my_command(robot) -> Command:
            return print("Hello World"!)
        ```
        """

        def _dec(func):
            # This also prevents segfaults. Took an hour to figure out. Thanks C++.
            if name in cls._options:
                raise ValueError(f"Route with name '{name}' already exists")

            cls._instance.addOption(name, func(cls._instance.robot))
            cls._options.add(name)

            return func

        return _dec
