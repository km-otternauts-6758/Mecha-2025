import wpilib
from rev import SparkMax
from wpimath.controller import PIDController

kP = 0.50
kI = 0.00
kD = 0.03


class Wrist:
    def __init__(self, MotorChannel: int) -> None:
        self.wrist = SparkMax(MotorChannel, SparkMax.MotorType.kBrushless)
        self.wristEncoder = self.wrist.getEncoder()

        self.wristPid = PIDController(kP, kI, kD)
        self.wrist.setInverted(True)

    def set(self, speed: float) -> None:
        self.wrist.set(speed)

    def setPosition(self, position: float):
        self.wristEncoder.setPosition(position)

    def getPosition(self) -> float:
        return self.wristEncoder.getPosition()

    def calculate(self, measurement: float, setPoint: float) -> float:
        return self.wristPid.calculate(measurement, setPoint)

    def setIntegratorRange(self, min: float, max: float) -> None:
        self.wristPid.setIntegratorRange(min, max)
