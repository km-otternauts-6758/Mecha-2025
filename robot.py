import wpilib
import wpimath.controller
from wpimath.controller import (
    PIDController,
    ElevatorFeedforward,
    ProfiledPIDController,
    SimpleMotorFeedforwardMeters,
    ArmFeedforward,
)
from wpimath.trajectory import TrapezoidProfile
from wpilib import (
    SendableChooser,
    SmartDashboard,
    DutyCycleEncoder,
    CAN,
    DigitalInput,
    shuffleboard,
)
from wpiutil import SendableRegistry

from networktables import NetworkTables

from constants import Constants
from subsystems import drivetrain
from subsystems import recipmotors
from subsystems import wrist
from subsystems import smartmotioncontroller

import rev
from rev import (
    SparkMax,
    ClosedLoopConfig,
    ClosedLoopConfigAccessor,
    SparkMaxAlternateEncoder,
)

shoulderkP = 0.5
shoulderkI = 0.05
shoulderkD = 0.15

elevatorkP = 0.05
elevatorkI = 0.003
elevatorkD = 0.005

drivekP = 0
drivekI = 0
drivekD = 0

wristkP = 0.2
wristkI = 0.02
wristkD = 0.01

wristkS = 0
wristkG = 0
wristkV = 0.4
wristkA = 0

kS = 0
kG = 0
kV = 0.4
kA = 0

kLEDBuffer = 60


class MyRobot(wpilib.TimedRobot):
    def robotInit(self):
        """Robot initialization function"""

        # LEDS
        self.led = wpilib.AddressableLED(9)
        self.ledData = [wpilib.AddressableLED.LEDData() for _ in range(kLEDBuffer)]
        self.rainbowFirstPixelHue = 0
        self.led.setLength(kLEDBuffer)
        self.led.setData(self.ledData)
        self.led.start()

        # TESTING DASHBOARD FOR TUNING PID
        self.enableLiveWindowInTest(True)

        # DriveTrain
        self.drive = drivetrain.DriveTrain(2, 3, 15, 13)

        self.drivePid = PIDController(drivekP, drivekI, drivekD)

        # Xbox Controller.
        self.stick = wpilib.XboxController(0)

        # Added Limelight
        NetworkTables.initialize(server="10.67.58.2")
        self.limeLight = NetworkTables.getTable("limelight-kmrobot")
        self.tx = self.limeLight.getEntry("tx")

        # Elevator
        self.elevator = SparkMax(1, SparkMax.MotorType.kBrushless)
        self.elevator.setInverted(True)
        self.elevatorEncoder = self.elevator.getEncoder()
        # self.elevatorEncoder = self.elevator.getAbsoluteEncoder()
        # Elevator PID
        self.elevatorPid = ProfiledPIDController(
            elevatorkP, elevatorkI, elevatorkD, TrapezoidProfile.Constraints(5, 10)
        )
        self.elevatorFeedForward = ElevatorFeedforward(kS, kG, kV, kA)
        # self.elevatorDutyCycle = wpilib.DutyCycle(wpilib.DigitalInput(0)

        # Create Gyro
        # self.gyro = navx.AHRS.create_spi()

        # Create Limit Switch
        self.limitSwitch = DigitalInput(2)

        # Shoulder
        self.shoulder = SparkMax(5, SparkMax.MotorType.kBrushless)
        self.shoulderEncoder = self.shoulder.getEncoder()
        # self.shoulder.setInverted(True)

        # Shoulder PID
        self.shoulderPid = PIDController(shoulderkP, shoulderkI, shoulderkD)
        self.shoulderPid.setTolerance(2, 0)

        # # SHOULDER DUTY CYCLE ENCODER
        # self.dutyCycle = wpilib.DutyCycle(wpilib.DigitalInput(1))

        # # Intake
        self.intake = SparkMax(11, SparkMax.MotorType.kBrushless)

        # Wrist
        # self.wrist = wrist.Wrist(6)
        self.wrist = SparkMax(6, SparkMax.MotorType.kBrushless)
        self.wristEncoder = self.wrist.getEncoder()
        # Wrist PID
        self.wristPid = PIDController(wristkP, wristkI, wristkD)
        # Feedforward
        self.wristFeedForward = ArmFeedforward(wristkS, wristkG, wristkV, wristkA)

        # CLimber
        self.climb = SparkMax(9, SparkMax.MotorType.kBrushless)

    def robotPeriodic(self):
        self.rainbow()
        self.led.setData(self.ledData)

    def teleopInit(self):
        self.drive.setDeadBand(0.2)
        self.elevatorEncoder.setPosition(0)
        self.shoulderEncoder.setPosition(0)
        # self.wrist.setPosition(0)

        self.rainbow()
        self.led.setData(self.ledData)

    def teleopPeriodic(self):
        # Dead Band
        self.drive.setDeadBand(0.2)

        # Print Values
        # print("X-Offset", self.tx)
        # print("ShoulderEncoder", self.shoulderEncoder.getPosition())
        # print("PidGoal", self.elevatorPid.getGoal())
        # print("WinchEncoder", self.elevatorEncoder.getPosition())
        print("WristEncoder", self.wristEncoder.getPosition())
        # print("AtSetpoint", self.shoulderPid.atSetpoint())
        # print("Heat", self.intake.getMotorTemperature())
        SmartDashboard.putData("Elevator PID", self.elevatorPid)

        # # """Runs the motors with Mecanum drive."""
        self.speed = self.stick.getLeftY()

        self.turn = -self.stick.getLeftX()

        self.strafe = -self.stick.getRightX()

        self.drive.driveCartesian(self.speed, self.strafe, self.turn)
        # , self.gyro.getRotation2d()

        # # TRAPEZOIDAL PROFILE TEST, TEST THIS
        # if self.stick.getPOV() == 0:
        #     self.elevatorPid.setGoal(5)
        # self.elevator.set(
        #     self.elevatorPid.calculate(self.elevatorEncoder.getPosition())
        # )
        # self.elevator.set(self.elevatorPid.calculate(self.elevatorEncoder.getPosition()) + self.elevatorFeedForward)

        # SHOULDER & ELEVATOR CONTROLS
        # if self.stick.getPOV() == 0:
        #     self.elevator.set(
        #         self.elevatorPid.calculate(self.elevatorEncoder.getPosition(), 82.1)
        #     )
        #     self.shoulder.set(
        #         self.shoulderPid.calculate(self.shoulderEncoder.getPosition(), 44.71)
        #     )
        # elif self.stick.getPOV() == 90:
        #     self.elevator.set(
        #         self.elevatorPid.calculate(self.elevatorEncoder.getPosition(), 45.61)
        #     )
        # elif self.stick.getPOV() == 180:
        #     self.elevator.set(
        #         self.elevatorPid.calculate(self.elevatorEncoder.getPosition(), 21.309)
        #     )
        # elif self.stick.getPOV() == 270:
        #     self.elevator.set(
        #         self.elevatorPid.calculate(self.elevatorEncoder.getPosition(), 20)
        #     )
        #     self.shoulder.set(
        #         self.shoulderPid.calculate(self.shoulderEncoder.getPosition(), 44.71)
        #     )
        # # TEST PRESET: TEST FEEDFORWARD
        # elif self.stick.getRawButton(7):
        #     self.shoulder.set(
        #         self.shoulderPid.calculate(
        #             self.shoulderEncoder.getPosition(),
        #             59 + self.elevatorFeedForward,
        #         )
        #     )
        #     self.elevator.set(
        #         self.elevatorPid.calculate(self.elevatorEncoder.getPosition(), 0)
        #     )
        # elif self.stick.getRawButton(5):
        #     self.shoulder.set(-0.2)

        # elif self.stick.getRawButton(6):
        #     self.shoulder.set(0.2)

        # elif self.stick.getRawAxis(3):
        #     self.elevator.set(-0.3)

        # elif self.stick.getRawAxis(2):
        #     self.elevator.set(0.3)

        # else:
        #     self.elevator.set(0)
        #     self.shoulder.set(0)

        if self.stick.getRawButton(1):
            self.shoulder.set(-0.3)
        elif self.stick.getRawButton(4):
            self.shoulder.set(0.3)
        else:
            self.shoulder.set(0)

        if self.stick.getRawButton(3):
            self.elevator.set(-0.3)
        elif self.stick.getRawButton(2):
            self.elevator.set(0.3)
        else:
            self.elevator.setVoltage(-self.elevatorFeedForward.calculate(0.07))

        if self.stick.getRawButton(8):
            self.intake.set(-0.6)
        elif self.stick.getRawButton(7):
            self.intake.set(0.6)
        else:
            self.intake.set(0)

        # # STOP POINT
        # if self.shoulderPid.atSetpoint() == True:
        #     self.shoulder.set(0)

        # ALIGNLIGHT, NEEDS TESTING
        # if self.stick.getRawButton(10):
        #     self.drive.set(self.elevatorPid.calculate(self.tx, 0))

        if self.stick.getRawButton(5):
            # self.wrist.set(0.1)
            self.wrist.set(
                self.wristPid.calculate(self.wristEncoder.getPosition(), 10.6)
            )
        # elif self.stick.getRawButton(6):
        # self.wrist.set(0.1)
        # self.wrist.set(
        #     self.wristPid.calculate(self.wristEncoder.getPosition(), 11.4)
        # )
        else:
            self.wrist.set(
                self.wristPid.calculate(self.wristEncoder.getPosition(), 11.4)
            )
        # self.wrist.set(self.wristFeedForward.calculate(0.03, 0.03))
        # self.wrist.set(self.wristPid.calculate(self.wrist.getPosition(), 0.0))

        # # WRIST CONTROLS - Test Feedforward
        # if self.stick.getRawButton(3):
        #     # self.wrist.setIntegratorRange(0, 1)
        #     # self.feedforward.calculate(10, 20)
        #     self.wrist.set(self.wrist.calculate(self.wrist.getPosition(), 0.0))
        # elif self.stick.getRawButton(2):
        #     # self.wrist.setIntegratorRange(0, 1)
        #     self.wrist.set(
        #         self.wrist.calculate(self.wrist.getPosition(), -0.238)
        #         + self.wristFeedForward
        #     )
        # else:
        #     self.wrist.set(0)

        # LIMIT SWITCH
        # if self.limitSwitch.get():
        #    self.elevatorEncoder.setPosition(0)

        # # # WRIST PID STUFF

        # SHOULDER SETPOINT BUTTONS
        # if self.stick.getRawButton(5):
        #     self.shoulder.set(
        #         self.shoulderPid.calculate(self.shoulderEncoder.getPosition(), 37)
        #     )
        # if self.stick.getRawButton(6):
        #     self.shoulder.set(
        #         self.shoulderPid.calculate(self.shoulderEncoder.getPosition(), 68)
        #     )
        # elif self.stick.getRawButton(5):
        #     self.shoulder.set(-0.2)
        # elif self.stick.getRawButton(6):
        #     self.shoulder.set(0.2)
        # else:
        #     self.shoulder.set(0)

        # INTAKE
        # if self.stick.getRawButton(4):
        #     self.intake.set(-0.45)
        # else:
        #     self.intake.set(0)

        # CLIMBER, Bumper Buttons for testing, Change them later.
        # if self.stick.getRawButton(5):
        #     self.climb.set(-1)
        # elif self.stick.getRawButton(6):
        #     self.climb.set(1)
        # else:
        #     self.climb.set(0)

    def rainbow(self):
        for i in range(kLEDBuffer):
            hue = (self.rainbowFirstPixelHue + (i * 180 / kLEDBuffer)) % 180
            self.ledData[i].setHSV(int(hue), 255, 128)
        self.rainbowFirstPixelHue += 3
        self.rainbowFirstPixelHue %= 180
