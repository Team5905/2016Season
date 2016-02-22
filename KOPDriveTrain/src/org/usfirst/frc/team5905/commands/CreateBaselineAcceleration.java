package org.usfirst.frc.team5905.commands;

import org.usfirst.frc.team5905.Robot;
import org.usfirst.frc.team5905.subsystems.DriveTrain;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class CreateBaselineAcceleration extends Command {
	
	private double numberOfExecutions = 0;
	private double sumOfAcceleration = 0;

    public CreateBaselineAcceleration(double timeout) {
    	setTimeout(timeout);
    	requires(Robot.driveTrain);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	numberOfExecutions++;
    	sumOfAcceleration += Robot.driveTrain.getRawAccelForwardDirectionVal();
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return isTimedOut();
    }

    // Called once after isFinished returns true
    protected void end() {
    	Robot.driveTrain.accelBaseline = (sumOfAcceleration/numberOfExecutions);
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
