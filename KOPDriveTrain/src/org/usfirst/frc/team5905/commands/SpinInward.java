package org.usfirst.frc.team5905.commands;

import org.usfirst.frc.team5905.Robot;
import org.usfirst.frc.team5905.RobotMap;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class SpinInward extends Command {

    public SpinInward() {
        // Use requires() here to declare subsystem dependencies
    	requires(Robot.intake);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	Robot.intake.stopSpin();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	if (Robot.oi.gamepad.getRawAxis(RobotMap.RIGHT_GAMEPAD_TRIGGER) > 0.5)
    		Robot.intake.spinInward();
    	else
    		Robot.intake.stopSpin();
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
