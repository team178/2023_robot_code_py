import wpilib

from robot import Robot


class RobotEntry(Robot):
    """
    What's this class for?

    This class is for tricking PyFRC, which handles the deployment of robot code for robotpy,
    into uploading this code properly. The way it was written, instead of being configurable
    or flexible in what code can be uploaded, it instead grabs the filepath of the file the
    Robot class you run is created, then uploads everything recursively in that folder. This
    is annoying because it destroys all of my package structure, and the package 'robot' no
    longer exists.

    This is a way to get around that, by making a class that inherits from the actual Robot
    class, giving it the exact same functionality, but giving PyFRC a different filepath. It
    will hopefully now upload the entire package.

    This file is used as an entrypoint for the actual robot code.
    """

    pass


wpilib.run(RobotEntry)
