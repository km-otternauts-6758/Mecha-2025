import wpilib
from wpimath.controller import PIDController

from networktables import NetworkTables

# from subsystems.limelight import LimeLight
from constants import Constants
from subsystems import drivetrain
from subsystems import recipmotors
from subsystems import wrist

from wpilib.shuffleboard import Shuffleboard

from wpilib import SendableChooser, SmartDashboard, DutyCycleEncoder, CAN, DigitalInput

import rev
from rev import (
    SparkMax,
    ClosedLoopConfig,
    ClosedLoopConfigAccessor,
    SparkMaxAlternateEncoder,
)

shoulderkP = 0
shoulderkI = 0
shoulderkD = 0

elevatorkP = 0.05
elevatorkI = 0.003
elevatorkD = 0.005


class MyRobot(wpilib.TimedRobot):
    def robotInit(self):
        """Robot initialization function"""
        # DriveTrain
        self.drive = drivetrain.DriveTrain(4, 2, 1, 3)

        # Xbox Controller.
        self.stick = wpilib.XboxController(0)

        # Added Limelight
        NetworkTables.initialize(server="10.67.58.2")
        self.limeLight = NetworkTables.getTable("limelight-kmrobot")
        self.tx = self.limeLight.getEntry("tx")

        # Create Elevator
        self.elevator = SparkMax(9, SparkMax.MotorType.kBrushless)
        self.elevator.setInverted(True)
        self.elevatorEncoder = self.elevator.getEncoder()

        # Create Gyro
        # self.gyro = navx.AHRS.create_spi()

        # Create Limit Switch
        self.limitSwitch = DigitalInput(0)

        # Shuffleboard
        tab = Shuffleboard.getTab("Pid")
        tab.addNumber
        # Shoulder
        # self.shoulder = SparkMax(5, SparkMax.MotorType.kBrushless)
        # self.shoulder.setInverted(True)

        # # Intake
        # self.intake = SparkMax(6, SparkMax.MotorType.kBrushless)

        # # Wrist
        # self.wrist = wrist.Wrist(7)
        # self.wristSetPoint = 0.4

        # DUTY CYCLE ENCODER - make sure sparkmax is set to Alternate Encoder in the client
        self.dutyCycle = wpilib.DutyCycle(wpilib.DigitalInput(1))

        self.elevatorDutyCycle = wpilib.DutyCycle(wpilib.DigitalInput(0))

        # PID STUFF
        # self.shoulderPid = PIDController(shoulderkP, shoulderkI, shoulderkD)
        # self.shoulderSetpoint = 0.4

        self.elevatorPid = PIDController(elevatorkP, elevatorkI, elevatorkD)
        self.elevatorSetpoint = 40

    def teleopInit(self):
        self.drive.setDeadBand(0.99)

        self.elevatorEncoder.setPosition(0)

    def teleopPeriodic(self):
        self.drive.setDeadBand(0.99)
        print(self.tx)
        print("boreEncoder", self.dutyCycle.getOutput())
        print("PID", self.elevatorPid.getError())
        print("WinchEncoder", self.elevatorEncoder.getPosition())

        # """Runs the motors with Mecanum drive."""
        self.speed = self.stick.getLeftY()

        self.turn = -self.stick.getLeftX()

        self.strafe = -self.stick.getRightX()

        self.drive.driveCartesian(self.speed, self.strafe, self.turn)
        # , self.gyro.getRotation2d()

        # # SHOULDER PID STUFF
        # self.shoulder.set(
        #     self.shoulderPid.calculate(
        #         self.dutyCycle.getOutput(), self.shoulderSetpoint
        #     )
        # )

        # ELEVATOR PID STUFF

        # self.elevator.set(0.3)

        # # Elevator Manual Control
        # if self.stick.getRawButton(6):
        #     self.elevator.set(0.5)

        # elif self.stick.getRawButton(4):
        #     self.elevator.set(-0.5)

        # else:
        #     self.elevator.set(0)

        # POV ELEVATOR CONTROLS
        if self.stick.getPOV() == 0:
            self.elevator.set(
                self.elevatorPid.calculate(self.elevatorEncoder.getPosition(), 80)
            )
        elif self.stick.getPOV() == 90:
            self.elevator.set(
                self.elevatorPid.calculate(self.elevatorEncoder.getPosition(), 60)
            )
        elif self.stick.getPOV() == 180:
            self.elevator.set(
                self.elevatorPid.calculate(self.elevatorEncoder.getPosition(), 40)
            )
        elif self.stick.getPOV() == 270:
            self.elevator.set(
                self.elevatorPid.calculate(self.elevatorEncoder.getPosition(), 20)
            )
        elif self.stick.getRawButton(7):
            self.elevator.set(
                self.elevatorPid.calculate(self.elevatorEncoder.getPosition(), 0)
            )
        elif self.stick.getRawButton(4):
            self.elevator.set(-0.5)
        elif self.stick.getRawButton(6):
            self.elevator.set(0.5)
        else:
            self.elevator.set(0)

        if self.limitSwitch.get():
            self.elevatorEncoder.setPosition(0)
        # # # WRIST PID STUFF
        # self.wrist.set(
        #     self.wrist.wristPid.calculate(
        #         self.wrist.wristEncoder.getPosition(), self.wristSetPoint
        #     )
        # )

        # # SHOULDER SETPOINT BUTTONS
        # if self.stick.getRawButton(1):
        #     self.shoulderSetpoint = 0.5

        # if self.stick.getRawButton(2):
        #     self.shoulderSetpoint = 0.6

        # if self.stick.getRawButton(3):
        #     self.shoulderSetpoint = 0.7

        # # ELEVATOR SETPOINT BUTTONS
        # if self.stick.getPOV(0):
        #     self.elevatorSetpoint = 0.5

        # if self.stick.getPOV(90):
        #     self.elevatorSetpoint = 0.6

        # if self.stick.getPOV(180):
        #     self.elevatorSetpoint = 0.7

        # if self.stick.getPOV(270):
        #     self.elevatorSetpoint = 0.8

        # # INTAKE
        # if self.stick.getRightTriggerAxis() >= 0.9:
        #     self.intake.set(0.1)
        # else:
        #     self.intake.set(0)
