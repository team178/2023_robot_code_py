import functools
from commands2 import Command, Subsystem, cmd


def run(func):
    """
    Utility decorator for wrapping a function to make it return a Command object
    without having to write it into the function itself. Uses the `run()` command
    function.
    """

    @functools.wraps(func)
    def _cmd(*args, **kwargs) -> Command:
        requires = []

        # * If part of a class that inherits from Subsystem, require it automatically
        if args and isinstance(args[0], Subsystem):
            requires = [args[0]]

        return cmd.run(lambda: func(*args, **kwargs), requirements=requires)

    return _cmd


def run_once(func):
    """
    Utility decorator for wrapping a function to make it return a Command object
    without having to write it into the function itself. Uses the `run_once()` command
    function.
    """

    @functools.wraps(func)
    def _cmd(*args, **kwargs) -> Command:
        requires = []

        # * If part of a class that inherits from Subsystem, require it automatically
        if args and isinstance(args[0], Subsystem):
            requires = [args[0]]

        return cmd.runOnce(lambda: func(*args, **kwargs), requirements=requires)

    return _cmd
