from commands2 import SubsystemBase
from wpilib import Solenoid, PneumaticsModuleType, DigitalInput

from robot.constants import *
import robot.util.cmd as cmd


class Claw(SubsystemBase):
    _solenoid = Solenoid(PneumaticsModuleType.CTREPCM, CLAW_CHANNEL)
    _photosensor = DigitalInput(CLAW_PHOTO)

    def __init__(self):
        super().__init__()
        self._solenoid.set(False)

    @cmd.run_once
    def toggle(self):
        self._solenoid.toggle()
