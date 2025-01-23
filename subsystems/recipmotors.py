# from rev import SparkMax, SparkMaxConfig, SparkMaxConfigAccessor

# #NEED TO TEST THIS, MIGHT NOT WORK, I DON'T KNOW CONFIG

# class ReciprocalMotors:
#     def __init__(self, leftMotorChannel: int, rightMotorChannel: int) -> None:
#         self.leftMotor = SparkMax(leftMotorChannel, SparkMax.MotorType.kBrushless)
#         self.rightMotor = SparkMax(rightMotorChannel, SparkMax.MotorType.kBrushless)

#         self.rightconfig = SparkMaxConfig()
#         self.rightconfig.follow(leftMotorChannel, False)
#     def set(self, speed: float) -> None:
#         self.leftMotor.set(speed)
