import wpilib

class SparkMaxRelativeEncoder:
    def __init__(self) -> None:
        self._velocity = 0

    def getVelocity(self):
        return self._velocity

class CANSparkMax(wpilib.Spark):
    def __init__(self, channel: int, ignored):
        super().__init__(channel - 8) # this is because we can't go to 20 on PWMSim
        self._encoder = SparkMaxRelativeEncoder()
    
    def restoreFactoryDefaults(self):
        pass

    def setInverted(self, b):
        pass

    def getEncoder(self):
        return self._encoder

    def setIdleMode(self, mode):
        pass
    
    class IdleMode:
        kBrake = 0
        kCoast = 1

    class MotorType:
        kBrushed = 0
        kBrushless = 1
