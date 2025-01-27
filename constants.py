import math


class Constants:
    stickDeadband = 0
    trackWidth = 0.6096  # 24 Inches
    wheelBase = 0.1524
    wheelDiameter = 0.1524  # 6 Inches
    wheelCircumference = math.pi * wheelDiameter
    driveGearRatio = 18 / 1
    conversionfactor = wheelCircumference / driveGearRatio
    wristSetpoint = 0.4
