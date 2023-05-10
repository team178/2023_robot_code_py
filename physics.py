import math

from wpimath.system.plant import DCMotor
from pyfrc.physics.core import PhysicsInterface
from wpilib.simulation import SingleJointedArmSim, DIOSim, SimDeviceSim

from robot.subsystems.arm import ArmPosition
from robot.constants import *


class PhysicsEngine:
    """
    Physics simulation setup
    """

    def __init__(self, physics_controller: PhysicsInterface, robot: "Robot"):
        self.physics_controller = physics_controller

        self.arm_gearbox = DCMotor.NEO()

        self.lower_arm_sim = SingleJointedArmSim(
            gearbox=self.arm_gearbox,
            gearing=100,
            moi=SingleJointedArmSim.estimateMOI(0.762, 4),
            armLength=0.762,
            minAngle=0,
            maxAngle=math.radians(360),
            simulateGravity=True,
        )

        self.upper_arm_sim = SingleJointedArmSim(
            gearbox=self.arm_gearbox,
            gearing=100,
            moi=SingleJointedArmSim.estimateMOI(0.659, 3),
            armLength=0.659,
            minAngle=0,
            maxAngle=math.radians(180),
            simulateGravity=True,
        )

        self.lower_encoder_pos = SimDeviceSim(
            "DutyCycle:DutyCycleEncoder", LOWER_ARM_ENCODER
        ).getDouble("absPosition")
        self.upper_encoder_pos = SimDeviceSim(
            "DutyCycle:DutyCycleEncoder", UPPER_ARM_ENCODER
        ).getDouble("absPosition")

        self.l_spark = SimDeviceSim("SPARK MAX ", LOWER_ARM_MOTOR)
        self.l_spark_output = self.l_spark.getDouble("Applied Output")

        self.u_spark = SimDeviceSim("SPARK MAX ", UPPER_ARM_MOTOR)
        self.u_spark_output = self.u_spark.getDouble("Applied Output")

        self.l_limit = DIOSim(LOWER_ARM_HOME)
        self.u_limit = DIOSim(UPPER_ARM_HOME)

    def update_sim(self, now: float, tm_diff: float) -> None:
        self.lower_arm_sim.setInputVoltage(self.l_spark_output.get())
        self.upper_arm_sim.setInputVoltage(self.u_spark_output.get())

        self.lower_arm_sim.update(tm_diff)
        self.upper_arm_sim.update(tm_diff)

        self.lower_encoder_pos.set(
            (ArmPosition.HOME[0] - self.lower_arm_sim.getAngle()) / math.radians(360)
        )
        self.upper_encoder_pos.set(
            (ArmPosition.HOME[1] - self.upper_arm_sim.getAngle()) / math.radians(360)
        )

        self.l_limit.setValue(
            self.lower_arm_sim.wouldHitLowerLimit(self.lower_arm_sim.getAngle())
        )
        self.u_limit.setValue(
            self.upper_arm_sim.wouldHitLowerLimit(self.lower_arm_sim.getAngle())
        )
