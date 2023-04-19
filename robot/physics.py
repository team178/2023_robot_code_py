import math

from wpilib import RobotController
from wpimath.system.plant import DCMotor
from pyfrc.physics.core import PhysicsInterface
from wpilib.simulation import SingleJointedArmSim, DutyCycleEncoderSim, SimDeviceSim, PWMSim

from robot.subsystems.arm import ArmPosition
from robot.constants import * 

class PhysicsEngine:

    def __init__(self, physics_controller: PhysicsInterface, robot: "Robot"):
        
        self.physics_controller = physics_controller

        self.arm_gearbox = DCMotor.NEO()

        self.lower_arm_sim = SingleJointedArmSim(
            gearbox = self.arm_gearbox,
            gearing = 100,
            moi = SingleJointedArmSim.estimateMOI(0.63, 9.0),
            armLength = 0.63,
            minAngle = math.radians(0),
            maxAngle = math.radians(360),
            simulateGravity = True
        )
        
        self.lower_encoder_sim = DutyCycleEncoderSim(LOWER_ARM_ENCODER)
        self.lower_encoder_sim.setDistance(ArmPosition.HOME[0]/math.radians(360))
        self.lower_motor = PWMSim(0)

        self.l_spark = SimDeviceSim("SPARK MAX", 20)
        self.l_spark_output = self.l_spark.getDouble('Applied Output')

    def update_sim(self, now: float, tm_diff: float) -> None:

        self.l_spark_output.set(self.lower_motor.getSpeed())

        self.lower_arm_sim.setInput(
            0, self.lower_motor.getSpeed() * RobotController.getInputVoltage()
        )

        self.lower_arm_sim.update(tm_diff)

        self.lower_encoder_sim.setDistance(self.lower_arm_sim.getAngle())
