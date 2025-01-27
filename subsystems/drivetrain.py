from rev import SparkMax
import wpilib
from wpilib.drive import MecanumDrive
from wpilib import SendableChooser
from wpimath.geometry import Rotation2d, Translation2d
from wpimath.kinematics import (
    ChassisSpeeds,
    MecanumDriveKinematics,
    MecanumDriveWheelSpeeds,
)
import math
from rev import SparkMax, EncoderConfig, SparkMaxConfig, SparkMaxConfigAccessor
import wpilib.shuffleboard
from constants import Constants


class DriveTrain:
    def __init__(
        self,
        frontleftMotorChannel: int,
        rearleftMotorChannel: int,
        frontrightMotorChannel: int,
        rearrightMotorChannel: int,
    ) -> None:
        # Motorz
        self.frontLeftMotor = SparkMax(
            frontleftMotorChannel, SparkMax.MotorType.kBrushless
        )
        self.rearLeftMotor = SparkMax(
            rearleftMotorChannel, SparkMax.MotorType.kBrushless
        )
        self.frontRightMotor = SparkMax(
            frontrightMotorChannel, SparkMax.MotorType.kBrushless
        )
        self.rearRightMotor = SparkMax(
            rearrightMotorChannel, SparkMax.MotorType.kBrushless
        )

        self.frontLeftMotor.setInverted(True)
        self.frontRightMotor.setInverted(False)
        self.rearLeftMotor.setInverted(True)
        self.rearRightMotor.setInverted(False)

        # Encoderz
        # self.frontLeftEncoder = self.frontLeftMotor.getEncoder()
        # self.rearLeftEncoder = self.rearLeftMotor.getEncoder()
        # self.frontRightEncoder = self.frontRightMotor.getEncoder()
        # self.rearRightEncoder = self.rearRightMotor.getEncoder()

        # self.DriveEncoderConfig = EncoderConfig()
        # self.DriveEncoderConfig.positionConversionFactor(Constants.conversionfactor)
        # self.DriveEncoderConfig.velocityConversionFactor(
        #     Constants.conversionfactor / 60
        # )

        self.drive = MecanumDrive(
            self.frontLeftMotor,
            self.rearLeftMotor,
            self.frontRightMotor,
            self.rearRightMotor,
        )

        # # Locations of the wheels relative to the robot center.
        # frontLeftLocation = Translation2d(0.330, 0.318)
        # frontRightLocation = Translation2d(0.330, -0.318)
        # backLeftLocation = Translation2d(-0.330, 0.318)
        # backRightLocation = Translation2d(-0.330, -0.318)

        # # Creating my kinematics object using the wheel locations.
        # self.kinematics = MecanumDriveKinematics(
        #     frontLeftLocation, frontRightLocation, backLeftLocation, backRightLocation
        # )

        # # The desired field relative speed here is 2 meters per second
        # # toward the opponent's alliance station wall, and 2 meters per
        # # second toward the left field boundary. The desired rotation
        # # is a quarter of a rotation per second counterclockwise. The current
        # # robot angle is 45 degrees.
        # speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
        #     2.0, 2.0, math.pi / 2.0, Rotation2d.fromDegrees(45.0)
        # )
        # # Now use this in our kinematics
        # wheelSpeeds = self.kinematics.toWheelSpeeds(speeds)

        # # Example wheel speeds - 19.68 inches per sec = 0.5m
        # wheelSpeeds = MecanumDriveWheelSpeeds(0.5, 0.5, 0.5, 0.5)
        # # Convert to chassis speeds
        # chassisSpeeds = self.kinematics.toChassisSpeeds(wheelSpeeds)
        # # Getting individual speeds
        # forward = ChassisSpeeds.vx
        # sideways = ChassisSpeeds.vy
        # angular = ChassisSpeeds.omega

    def driveCartesian(
        self, xSpeed: float, ySpeed: float, zRotation: float
    ):  # add gyroAngle for field orient.
        self.drive.driveCartesian(xSpeed, ySpeed, zRotation)

    # , gyroAngle: Rotation2d
    # , gyroAngle
    # def getDrivePosition(self) -> float:
    #     return (
    #         self.frontLeftEncoder.getPosition() + self.rearLeftEncoder.getPosition()
    #     ) / 2

    # def getDriveVelocity(self) -> float:
    #     return (
    #         self.frontLeftEncoder.getVelocity() + self.rearLeftEncoder.getVelocity()
    #     ) / 2
    def setDeadBand(self, deadband: float):
        self.drive.setDeadband(deadband)
