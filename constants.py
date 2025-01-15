from wpimath.units import inchesToMeters
import math

class Constants:
    def __init__(self):
        self.stickDeadband = 0.1
        self.trackWidth = inchesToMeters(24)
        self.wheelBase = inchesToMeters(24)
        self.wheelDiameter = inchesToMeters(4)
        self.wheelCircumference = math.pi * self.wheelDiameter
        self.driveGearRatio = 6.75 / 1 # 6.75:1
        self.angleGearRatio = 12.8 / 1 # 12.8:1