package org.usfirst.frc.team5905.commands;

import org.usfirst.frc.team5905.Robot;
import org.usfirst.frc.team5905.subsystems.DriveTrain;

import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class AutoStraightCommand extends Command {

    public AutoStraightCommand() {
    	
    //TODO: Change this
    	setTimeout(20);
    	// Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires(Robot.driveTrain);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	Robot.driveTrain.prepareToGoStraight();
    	
    }
    
    double counter = 0;

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	SmartDashboard.putNumber("AutoStraightCounter", counter+=1);
    	Robot.driveTrain.goStraight(1);
    	
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return isTimedOut();
    }

    // Called once after isFinished returns true
    protected void end() {
    	Robot.driveTrain.stop();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
