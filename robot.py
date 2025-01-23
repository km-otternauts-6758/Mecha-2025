import wpilib
from wpimath.controller import PIDController

from subsystems import drivetrain
from subsystems import recipmotors
from subsystems import elevator

from wpilib import SendableChooser, SmartDashboard, DutyCycleEncoder
from rev import (
    SparkMax,
    ClosedLoopConfig,
    ClosedLoopConfigAccessor,
    SparkMaxAlternateEncoder,
)

kP = 12
kI = 0
kD = 0
setpoint = 0.523


class MyRobot(wpilib.TimedRobot):
    def robotInit(self):
        """Robot initialization function"""
        # DriveTrain
        self.drive = drivetrain.DriveTrain(4, 2, 1, 3)

        # Define the Xbox Controller.
        self.stick = wpilib.XboxController(0)

        # Create Elevator
        # self.elevator = elevator.Elevator(10)
        self.elevator = SparkMax(10, SparkMax.MotorType.kBrushless)
        # self.elevatorEncoder = self.elevator.getEncoder()
        # self.elevatorEncoder
        # Create Gyro
        # self.gyro = navx.AHRS.create_spi()

        # Shoulder
        self.shoulder = SparkMax(5, SparkMax.MotorType.kBrushless)
        self.shoulder.setInverted(True)
        # self.shoulderEncoder = self.shoulder.getEncoder()

        # Intake
        self.intake = SparkMax(6, SparkMax.MotorType.kBrushless)
        # self.drive.frontLeftEncoder.

        # DUTY CYCLE ENCODER - make sure sparkmax is set to Alternate Encoder in the client
        self.dutyCycle = wpilib.DutyCycle(wpilib.DigitalInput(0))

        # PID SHIT
        # Creates a PIDController with gains kP, kI, and kD
        self.pid = PIDController(kP, kI, kD)
        # Enables continuous input on a range from -180 to 180
        # self.pid.enableContinuousInput(-180, 180)

    def teleopInit(self):
        self.drive.setDeadBand(0.5)

    def teleopPeriodic(self):
        # SmartDashboard.putNumber("boreEncoder", self.dutyCycle.getOutput())
        # SmartDashboard.putNumber("P", PIDController.getP())
        # SmartDashboard.putNumber("I", PIDController.getI())
        # SmartDashboard.putNumber("D", PIDController.getD())
        # SmartDashboard.putNumber("PID", self.pid.getError())
        # SmartDashboard.putNumber(
        #     "ShoulderOutputCurrent", self.shoulder.getOutputCurrent()
        # )

        print("boreEncoder", self.dutyCycle.getOutput())
        print("PID", self.pid.getError())
        print("test", self.pid.calculate(self.dutyCycle.getOutput(), setpoint))
        # """Runs the motors with Mecanum drive."""
        self.speed = self.stick.getLeftY()

        self.turn = -self.stick.getLeftX()

        self.strafe = -self.stick.getRightX()
        # # Use the joystick X axis for lateral movement, Y axis for forward movement, and Z axis for rotation.
        # # This sample does not use field-oriented drive, so the gyro input is set to zero.
        # # This Stick configuration is created by K.E. on our team.  Left stick Y axis is speed, Left Stick X axis is strafe, and Right Stick Y axis is turn.
        self.drive.driveCartesian(self.speed, self.strafe, self.turn)
        # , self.gyro.getRotation2d()
        # Make preset levels 1-4\, winch makes it stop so gotta make a down function
        # self.elevator.set(self.stick.getYButton())

        # ELEVATOR D-PAD/POV CONTROL
        if self.stick.getPOV() == -1:
            self.elevator.set(0)

        if self.stick.getPOV() == 0:
            self.elevator.set(1)

        if self.stick.getPOV() == 180:
            self.elevator.set(-1)

        # # SHOULDER SHIT
        # if self.stick.getRawButton(5):
        #     self.shoulder.set(0.4)
        # elif self.stick.getRawButton(6):
        #     self.shoulder.set(-0.4)
        # else:
        #     self.shoulder.set(0)

        self.shoulder.set(self.pid.calculate(self.dutyCycle.getOutput(), setpoint))
        # self.pid.enableContinuousInput(-180, 180)

        # Intake
        if self.stick.getRawButton(1):
            self.intake.set(0.1)
        elif self.stick.getRawButton(4):
            self.intake.set(-0.1)
        else:
            self.intake.set(0)

        # # HEX ENCODER STUFF FROM LAST YEAR, SAVING FOR LATER
        # if self.stick.getRawButton(6):
        #     self.shoulder.set(0.3)
        #     if DutyCycleEncoder.getOutput() == 0.9750:
        #         self.shoulder.set(0)
        # if self.stick.getRawButton(4):
        #     self.shoulder.set(0.3)
        #     if self.dutyCycle.getOutput() == 0.778:
        #         self.shoulder.set(0)
