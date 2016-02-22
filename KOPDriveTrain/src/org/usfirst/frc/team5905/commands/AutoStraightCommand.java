package org.usfirst.frc.team5905.commands;

import org.usfirst.frc.team5905.Robot;
import org.usfirst.frc.team5905.RobotMap;
import org.usfirst.frc.team5905.subsystems.DriveTrain;

import edu.wpi.first.wpilibj.ADXL362;
import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.interfaces.Accelerometer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class AutoStraightCommand extends Command {
	
	Timer timer = new Timer();
	
	private double oldAccelValue = 0;
	private double oldVelValue = 0;
	
	public double velocity = 0;
	public double distance = 0;
	public double deltaTime = 0.02; //seconds
	
	
	public double lStartTime;
	public double lEndTime;
	
	

    public AutoStraightCommand(double timeout) {
    	
    //TODO: Change this
    	setTimeout(timeout);
    	// Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires(Robot.driveTrain);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	Robot.driveTrain.prepareToGoStraight();
    	lStartTime = System.currentTimeMillis();
    }
   
    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	
    	double lEndTime = System.currentTimeMillis();
    	deltaTime = (lEndTime - lStartTime)/1000;
    	lStartTime = lEndTime;
    	
    	double accel = Robot.driveTrain.getAccelForwardDirectionVal();
    	
    	
    	
    	
    	velocity += (oldAccelValue + accel) * deltaTime * .5;
    	distance += (oldVelValue + velocity) * deltaTime * .5;
    	
    	oldVelValue = velocity;
    	oldAccelValue = Robot.driveTrain.getAccelForwardDirectionVal();
    	
    	SmartDashboard.putNumber("DeltaTime", deltaTime);
		SmartDashboard.putNumber("Acceleration", accel);
		SmartDashboard.putNumber("Velocity", velocity);
		SmartDashboard.putNumber("Distance", distance);
    	
    	
    	
    	
    	
    	
    	
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return isTimedOut();
    }

    // Called once after isFinished returns true
    protected void end() {
    	Robot.driveTrain.stop();
    	velocity = 0;
    	oldAccelValue = 0;
    	oldVelValue = 0;
    	SmartDashboard.putNumber("Final Distance", distance);
    	distance = 0;
    	RobotMap.driveTrainAccel = new ADXL362(Accelerometer.Range.k2G);
    	
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
