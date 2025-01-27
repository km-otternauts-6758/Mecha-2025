import wpilib
from rev import SparkMax
from wpimath.controller import PIDController

kP = 0
kI = 0
kD = 0


class Wrist:
    def __init__(self, MotorChannel: int) -> None:
        self.wrist = SparkMax(MotorChannel, SparkMax.MotorType.kBrushless)
        self.wristEncoder = self.wrist.getEncoder()

        self.wristPid = PIDController(kP, kI, kD)
        self.wristSetpoint = 0.4

    def set(self, speed: float) -> None:
        self.wrist.set(speed)
