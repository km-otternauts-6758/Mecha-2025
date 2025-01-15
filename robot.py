from rev import SparkMax
import wpilib
from wpilib.drive import MecanumDrive


class MyRobot(wpilib.TimedRobot):
    def robotInit(self):
        """Robot initialization function"""
        self.frontLeftMotor = SparkMax(4, SparkMax.MotorType.kBrushless)
        self.rearLeftMotor = SparkMax(1, SparkMax.MotorType.kBrushless)
        self.frontRightMotor = SparkMax(2, SparkMax.MotorType.kBrushless)
        self.rearRightMotor = SparkMax(3, SparkMax.MotorType.kBrushless)

        # invert the left side motors
        self.rearLeftMotor.setInverted(True)
        self.frontLeftMotor.setInverted(True)
        # you may need to change or remove this to match your robot
        self.frontRightMotor.setInverted(True)
        self.rearRightMotor.setInverted(True)
        
        self.drive = MecanumDrive(
            self.frontLeftMotor,
            self.rearLeftMotor,
            self.frontRightMotor,
            self.rearRightMotor,
        )
        # Define the Xbox Controller.
        self.stick = wpilib.XboxController(1)
        
        

    def teleopInit(self):
        self.drive.setSafetyEnabled(True)
        


    def teleopPeriodic(self):
        """Runs the motors with Mecanum drive."""
        self.speed = -self.stick.getLeftX()
           
        self.turn =  self.stick.getRightX()
            
        self.strafe = self.stick.getLeftY()
        # Use the joystick X axis for lateral movement, Y axis for forward movement, and Z axis for rotation.
        # This sample does not use field-oriented drive, so the gyro input is set to zero.
        # This Stick configuration is created by K.E. on our team.  Left stick Y axis is speed, Left Stick X axis is strafe, and Right Stick Y axis is turn.

        self.drive.driveCartesian(
            self.speed, self.turn, self.strafe
        )
