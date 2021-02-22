from wpilib.command import Command
from wpilib import Timer
import math
from wpilib import SmartDashboard
from networktables import NetworkTables

class AutonomousAnglePID(Command):
    """ This command rotates the robot's shooter hood motor to the given angle"""
    def __init__(self, robot, setpoint=None, timeout=None, source=None):
        """The constructor"""
        Command.__init__(self, name='auto_angle_pid') # le name

        self.requires(robot.shooter) # requires le shooter

        self.setpoint = setpoint # the destination point
        self.source = source  # sent directly to command or via dashboard

        if timeout is None:
            self.timeout = 5
        else:
            self.timeout = timeout
        
        self.setTimeout(self.timeout) # set timeout

        self.robot = robot # le robot

        self.tolerance = 0.1
        self.kp = 0.3;  self.kd = 0.1; self.kf = 0.1 # P, D, and I respectively (why is it f) 

        self.interval_span = 0.02 # i was informed this thing ran every 0.02 seconds

        self.error = 0; # le error
        self.prev_error = 0; # le previous error
        self.riemann_sum = 0 # riemann sum thing for calculating le integral

        self.max_power = 0.5 # max power or something

    def initialize(self):
        """Called just before this Command runs the first time."""

        self.start_time = round(Timer.getFPGATimestamp() - self.robot.enabled_time, 1) # time at which robot starts

        print("\n" + f"** Started {self.getName()} with setpoint {self.setpoint} at {self.start_time} s **")
        SmartDashboard.putString("alert", f"** Started {self.getName()} with setpoint {self.setpoint} at {self.start_time} s **") # logging

        if self.source == 'dashboard':
            self.setpoint = SmartDashboard.getNumber('hood_angle', 1) # pulls this number from dashboard if source is dashboard
        # may want to break if no valid setpoint is passed

        self.error = 0 # why are we setting these values again
        self.prev_error = 0
        self.riemann_sum = 0

    def execute(self):
        """Called repeatedly when this Command is scheduled to run""" 
        self.error = self.setpoint - self.robot.shooter.getAngle() # le error, i modified this from auto rotate 

        self.riemann_sum += self.error * self.interval_span # integral part
        self.moment_slope = (self.error - self.prev_error) / self.interval_span # derivative part

        self.power = self.kp * self.error + self.kf * self.riemann_sum + self.kd * self.moment_slope # plug P, D, I, and their coefficients into the magic formula

        self.prev_error = self.error # previous error is now current error

        if self.power > 0:
            self.power = min(self.max_power, self.power)
        else:
            self.power = max(-self.max_power, self.power) # i copied this from auto rotate

        self.robot.shooter.set_hood_motor(self.power) # sets the hood motor to that power

        #SmartDashboard.putNumber("error", self.error)

    def isFinished(self):
        """Make this return true when this Command no longer needs to run execute()"""
        # I know I could do this with a math.copysign, but this is more readable
        if self.setpoint > 0:
            return self.error <= self.tolerance or self.isTimedOut()
        else:
            return self.error >= -self.tolerance or self.isTimedOut() # i assume this works because i copied it

    def end(self, message='Ended'):
        """Called once after isFinished returns true"""
        end_time = round(Timer.getFPGATimestamp() - self.robot.enabled_time, 1) # end time

        print(f"** {message} {self.getName()} at {end_time} s after {round(end_time-self.start_time,1)} s **")
        SmartDashboard.putString("alert", f"** Ended {self.getName()} at {end_time} s after {round(end_time - self.start_time, 1)} s **") # logging

        self.robot.shooter.set_hood_motor(0) # cease the hood motor (set power to 0) 

    def interrupted(self): 
        """Called when another command which requires one or more of the same subsystems is scheduled to run.""" 
        end_time = round(Timer.getFPGATimestamp() - self.robot.enabled_time, 1) # end time

        self.end(message=f"** Interrupted {self.getName()} at {end_time} s after {round(end_time-self.start_time,1)} s **") # logging
