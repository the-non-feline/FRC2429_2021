from wpilib.command import Command
from wpilib import Timer

class DriveByJoystick(Command):
    """
    This allows Logitech gamepad to drive the robot. It is always running
    except when interrupted by another command.
    """

    def __init__(self, robot):
        Command.__init__(self, name='DriveByJoystick')
        self.requires(robot.drivetrain)
        self.robot = robot

    def initialize(self):
        """Called just before this Command runs the first time."""
        self.start_time = round(Timer.getFPGATimestamp() - self.robot.enabled_time, 1)
        print("\n" + f"** Started {self.getName()} at {self.start_time} s **", flush=True)


    def execute(self):
        """Called repeatedly when this Command is scheduled to run"""
        self.robot.drivetrain.drive.arcadeDrive(-self.robot.oi.stick.getRawAxis(1), 0.75*self.robot.oi.stick.getRawAxis(4))


    def isFinished(self):
        """Make this return true when this Command no longer needs to run execute()"""
        # we never let this command end - only get interrupted.  it is the default command so it auto-starts when
        # nothing else wants the drivetrain
        return False


    def end(self):
        """Called once after isFinished returns true"""
        self.robot.drivetrain.drive.arcadeDrive(0,0)
        print("\n" + f"** Ended {self.getName()} at {round(Timer.getFPGATimestamp() - self.robot.enabled_time, 1)} s **", flush=True)


    def interrupted(self):
        """Called when another command which requires one or more of the same subsystems is scheduled to run."""
        self.robot.drivetrain.drive.arcadeDrive(0,0)
        print("\n" + f"** Interrupted {self.getName()} at {round(Timer.getFPGATimestamp() - self.robot.enabled_time, 1)} s **", flush=True)