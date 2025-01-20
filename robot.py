import wpilib

from subsystems import drivetrain
from subsystems import recipmotors
from wpilib import SendableChooser, SmartDashboard
from rev import SparkMax

# import navx


class MyRobot(wpilib.TimedRobot):
    def robotInit(self):
        """Robot initialization function"""
        # DriveTrain
        self.drive = drivetrain.DriveTrain(4, 2, 1, 3)

        # Define the Xbox Controller.
        self.stick = wpilib.XboxController(0)

        # Create Elevator
        self.elevator = recipmotors.ReciprocalMotors(9, 10)

        # Create Gyro
        # self.gyro = navx.AHRS.create_spi()

        # Shoulder
        self.shoulder = SparkMax(5, SparkMax.MotorType.kBrushless)

        # Intake
        self.intake = SparkMax(6, SparkMax.MotorType.kBrushless)
        # self.drive.frontLeftEncoder.

    def teleopInit(self):
        pass

    def teleopPeriodic(self):
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
            self.elevator.set(0.5)

        if self.stick.getPOV() == 180:
            self.elevator.set(0.025)

        # SHOULDER SHIT
        if self.stick.getRawButton(5):
            self.shoulder.set(0.4)
        elif self.stick.getRawButton(6):
            self.shoulder.set(-0.4)
        else:
            self.shoulder.set(0)

        # Intake
        if self.stick.getRawButton(1):
            self.intake.set(0.1)
        elif self.stick.getRawButton(4):
            self.intake.set(-0.1)
        else:
            self.intake.set(0)

        # HEX ENCODER STUFF FROM LAST YEAR, SAVING FOR LATER
        # if self.joystick.getRawButton(6):
        #     self.shoulder.set(0.3)
        #     if self.dutyCycle.getOutput() == 0.9750:
        #         self.shoulder.set(0)
        # if self.joystick.getRawButton(4):
        #     self.shoulder.set(0.3)
        #     if self.dutyCycle.getOutput() == 0.778:
        #         self.shoulder.set(0)
