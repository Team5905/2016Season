package org.usfirst.frc.team5905.subsystems;

import org.usfirst.frc.team5905.Robot;
import org.usfirst.frc.team5905.RobotMap;
import org.usfirst.frc.team5905.commands.MoveWithJoysticks;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.PIDSubsystem;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class PIDDriveTrain extends PIDSubsystem {
	
	private final SpeedController speedController1 = RobotMap.driveTrainSpeedController1;
	private final SpeedController speedController2 = RobotMap.driveTrainSpeedController2;
	private final SpeedController speedController3 = RobotMap.driveTrainSpeedController3;
	private final SpeedController speedController4 = RobotMap.driveTrainSpeedController4;
	private final RobotDrive robotDrive41 = RobotMap.driveTrainRobotDrive21;
	private final ADXRS450_Gyro gyro = RobotMap.driveTrainGyro;
	
	private final double LeftForwardCalibration = 1;
	private final double RightForwardCalibration = 1;
	private final double LeftBackwardCalibration = 1;
	private final double RightBackwardCalibration = 1;
	
	private static final double PVAL = 2.0;
	private static final double IVAL = 0.0;
	private static final double DVAL = 0.0;
	private static final double GYRO_TOLERANCE = 5.0;
	
	private DriveTrainMode mode;
	
	private double targetTime = 0;
	private enum DriveTrainMode {
		MANUAL, DRIVE_STRAIGHT, ROTATE

	}
	
	private Timer timer;
	private double timeOut = 2.0;
	public double oldLeftSpeed = 0;
	public double oldRightSpeed = 0;
	
	public PIDDriveTrain() {
		super(PVAL, IVAL, DVAL);
		this.setOutputRange(-1, 1);
		this.setInputRange(-360, 360);
		this.disable();
		RobotMap.driveTrainGyro.reset();
		mode = DriveTrainMode.MANUAL;

		
		// Timer
		timer = new Timer();

	}

    
    // Put methods for controlling this subsystem
    // here. Call these from Commands.

    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    	setDefaultCommand(new MoveWithJoysticks());
    }


	@Override
	protected double returnPIDInput() {
		return getCorrectedAngle();	
	}
	
	@Override
	protected void usePIDOutput(double output) {
		applyPIDOutput(output);
	}
	
	private void applyPIDOutput(double output) {
		smoothDrive(output, -output);
	}


	// Initialize your subsystem here
	
	public void stop() {
		oldLeftSpeed = 0;
		oldRightSpeed = 0;
		smoothDrive(0.0, 0.0);		
	}
	
	private double getCorrectedAngle() {
		double angle =  RobotMap.driveTrainGyro.getAngle();
		int multiplier = 1;
				
		if (angle < 0) 
			multiplier = -1;		
		
		return multiplier * (Math.abs(angle) % 360);
	}

	private void smoothDrive(double left, double right) {
		// TODO: aceept a flag indicating that smoothDrive should use Gyro input and compensate
		oldLeftSpeed = RobotMath.ease(left * LeftForwardCalibration, oldLeftSpeed);
		oldRightSpeed = RobotMath.ease(right * RightForwardCalibration, oldRightSpeed);
		
		// TODO: read Gyro angle and use it to REDUCE power to one of the motors
		
		// if gyro if off by 0.5 do not bother
		// otherwise get a scaled value based on (say) 30degrees
		
		final double MAX_ANGLE_ERROR_EXPECTED = 30;
		final double MAX_DELTA_OF_SIDE = 0.25;
		final double ANGLE_ERROR_TOLERANCE = 0.5;
		
		double gyroAngle = gyro.getAngle();
		double gyroAngleAbsolute = Math.abs(gyroAngle);
		double compensationFactor = 1;
		
		//Does not allow correction for more than MAX_ANGLE_ERROR_EXPECTED
		if (gyroAngleAbsolute > MAX_ANGLE_ERROR_EXPECTED){
			gyroAngleAbsolute = MAX_ANGLE_ERROR_EXPECTED;
		}
		
		//Find an absolute compensation for the motor proportionate to the maximum angle
		if (gyroAngleAbsolute > ANGLE_ERROR_TOLERANCE) {
			compensationFactor = 1 - (gyroAngleAbsolute * MAX_DELTA_OF_SIDE / MAX_ANGLE_ERROR_EXPECTED);
		}
		
		
		if (gyroAngle < 0) 
			right = compensationFactor * oldRightSpeed;
		else 
			left = compensationFactor * oldLeftSpeed;

		
		SmartDashboard.putNumber("leftMotor", left);
		SmartDashboard.putNumber("rightMotor", right);
		SmartDashboard.putNumber("rawGyroValue", gyroAngle);

		driveCalibrated(left, right);
	}
	
	public void prepareToGoStraight() {
		mode = DriveTrainMode.DRIVE_STRAIGHT;
		gyro.reset();
		oldLeftSpeed = 0;
		oldRightSpeed = 0;
	}
	
	public void goStraight(double power){
		// TODO: send a flag indicating that smoothDrive should use Gyro input and compensate
		smoothDrive(power, power);				
	}
	
	

	public boolean isTimeUp(){
		return (timer.get() >= targetTime);
	}

	public void moveWithJoysticks() {
		double rawLeftAxis = -1 * Robot.oi.gamepad.getRawAxis(RobotMap.LEFT_GAMEPAD_JOYSTICK);
		double rawRightAxis = -1 * Robot.oi.gamepad.getRawAxis(RobotMap.RIGHT_GAMEPAD_JOYSTICK);
		driveCalibratedSquared(rawLeftAxis, rawRightAxis);
	}

	private void driveCalibrated(double leftRawPower, double rightRawPower) {
		driveCalibratedSquared(leftRawPower, rightRawPower, false);
	}

	private void driveCalibratedSquared(double leftRawPower, double rightRawPower) {
		driveCalibratedSquared(leftRawPower, rightRawPower, true);
	}

	private void driveCalibratedSquared(double leftRawPower, double rightRawPower, boolean squaredInputs) {
		double leftPower = getCalibratedLeftPower(leftRawPower);
		double rightPower = getCalibratedRightPower(rightRawPower);

		robotDrive41.tankDrive(leftPower, rightPower, squaredInputs);
	}

	private double getCalibratedRightPower(double givenPower) {
		if (givenPower < 0) {
			return givenPower * RightBackwardCalibration;
		} else {
			return givenPower * RightForwardCalibration;
		}
	}

	private double getCalibratedLeftPower(double givenPower) {
		if (givenPower < 0) {
			return givenPower * LeftBackwardCalibration;
		} else {
			return givenPower * LeftForwardCalibration;
		}
	}

	public void driveBoth(double power) {
		driveCalibratedSquared(power, power);

	}


	
    
    
}

