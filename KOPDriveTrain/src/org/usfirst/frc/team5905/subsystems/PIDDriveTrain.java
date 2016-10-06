package org.usfirst.frc.team5905.subsystems;

import org.usfirst.frc.team5905.Robot;
import org.usfirst.frc.team5905.RobotMap;
import org.usfirst.frc.team5905.commands.MoveWithJoysticks;

import edu.wpi.first.wpilibj.ADXL362;
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
	
	private final SpeedController FRONT_RIGHT_SPEED_CONTROLLER = RobotMap.FRONT_RIGHT_SPEED_CONTROLLER;
	private final SpeedController BACK_RIGHT_SPEED_CONTROLLER = RobotMap.BACK_RIGHT_SPEED_CONTROLLER;
	private final SpeedController FRONT_LEFT_SPEED_CONTROLLER = RobotMap.FRONT_LEFT_SPEED_CONTROLLER;
	private final SpeedController BACK_LEFT_SPEED_CONTROLLER = RobotMap.BACK_LEFT_SPEED_CONTROLLER;

	private final RobotDrive robotDrive41 = RobotMap.driveTrain41;
	private final ADXRS450_Gyro gyro = RobotMap.driveTrainGyro;
	private final ADXL362 accel = RobotMap.driveTrainAccel;
	
	private final double LeftForwardCalibration = 1;
	private final double RightForwardCalibration = 1;
	private final double LeftBackwardCalibration = 1;
	private final double RightBackwardCalibration = 1;
	
	private static final double PVAL = 0.0175;
	//Increase IVAL
	private static final double IVAL = 0.0006;
	private static final double DVAL = 0.004;
	private static final double GYRO_TOLERANCE = 0.2;
	
	public static double accelBaseline = 0;
	
	
	DriveTrainMode mode;
	public static boolean stop;
	
	private double targetTime = 0;
	
	private enum DriveTrainMode {
		MANUAL, DRIVE_STRAIGHT, ROTATE, DRIVE_BACKWARDS
	}
	
	private Timer timer;
	private double timeOut = 2.0;
	public double oldLeftSpeed = 0;
	public double oldRightSpeed = 0;
	
	public PIDDriveTrain() {
		//PID stuff
		super(PVAL, IVAL, DVAL);
		this.setOutputRange(-1, 1);
		this.setInputRange(-360, 360);
		
		this.disable();
		
		stop = false;
		
		RobotMap.driveTrainGyro.reset();
		mode = DriveTrainMode.MANUAL;

		timer = new Timer();
	}

    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    	setDefaultCommand(new MoveWithJoysticks());
    }
	
	protected double returnPIDInput() {
		return getCorrectedAngle();	
	}
	
	protected void usePIDOutput(double output) {
		applyPIDOutput(output);
	}
	
	private void applyPIDOutput(double output) {
		smoothDrive(output, -output);
	}
	
	public void stop() {
		oldLeftSpeed = 0;
		oldRightSpeed = 0;
		driveCalibrated(0.001, 0.001);
	}
	
	private double getCorrectedAngle() {
		double angle =  RobotMap.driveTrainGyro.getAngle();
		int multiplier = 1;
				
		if (angle < 0) 
			multiplier = -1;		
		
		return multiplier * (Math.abs(angle) % 360);
	}
	
	private void smoothDrive(double left, double right){
		smoothDriveDirection(left, right, 0);
	}
	
	private void smoothDriveForward(double left, double right){
		smoothDriveDirection(left, right, 1);
	}
	
	private void smoothDriveBackward(double left, double right){
		smoothDriveDirection(left, right, -1);
	}

	private void smoothDriveDirection(double left, double right, int direction) {
		oldLeftSpeed = RobotMath.ease(left * LeftForwardCalibration, oldLeftSpeed);
		oldRightSpeed = RobotMath.ease(right * RightForwardCalibration, oldRightSpeed);
				
		// if gyro if off by 0.5 degrees (indefinite value) do not bother
		// otherwise get a scaled value based on 30 degrees (indefinite value)
		
		if (mode == null) {
			driveCalibrated(0, 0);
		}
		
		if (mode == DriveTrainMode.DRIVE_STRAIGHT) {
			final double MAX_ANGLE_ERROR_EXPECTED = 3;
			final double MAX_DELTA_OF_SIDE = 0.5;
			final double ANGLE_ERROR_TOLERANCE = 0.1;
			
			double gyroAngle = gyro.getAngle();
			double gyroAngleAbsolute = Math.abs(gyroAngle);
			double compensationFactor = 1;
			
			//Does not allow correction for more than MAX_ANGLE_ERROR_EXPECTED
			
			if (gyroAngleAbsolute > MAX_ANGLE_ERROR_EXPECTED){
				gyroAngleAbsolute = MAX_ANGLE_ERROR_EXPECTED;
			}
			
			//Find an absolute compensation for the motor proportional to the maximum angle
			
			if (gyroAngleAbsolute > ANGLE_ERROR_TOLERANCE) {
				compensationFactor = 1 - (gyroAngleAbsolute * MAX_DELTA_OF_SIDE / MAX_ANGLE_ERROR_EXPECTED);
			}
			
			else {
				if (gyroAngle > 0) 
					right = compensationFactor * oldRightSpeed;
				else 
					left = compensationFactor * oldLeftSpeed;
			}
			
			if (direction != 0){
				right *= direction;
				left *= direction;
			}

		
		SmartDashboard.putNumber("leftMotor", left);
		SmartDashboard.putNumber("rightMotor", right);
		SmartDashboard.putNumber("rawGyroValue", gyro.getAngle());
		driveCalibrated(left, right);
	}
}
	
	
	public double getCurrentGyroAngle(){
		return gyro.getAngle();
	}
	
	public void prepareToGoStraight() {
		mode = DriveTrainMode.DRIVE_STRAIGHT;
		this.disable();
		gyro.reset();		
		
		oldLeftSpeed = 0;
		oldRightSpeed = 0;
	}
	
	public double getRawAccelForwardDirectionVal() {
		
		return vectorAddition(accel.getX(), accel.getY());
		
	}
	
	//TODO: Move to Calculations section
	public double vectorAddition (double a, double b){
		return 
				Math.sqrt(
				Math.pow(a, 2) +
				Math.pow(b, 2)
				);
	}
	
	public double getAccelForwardDirectionVal() {
		double multiplier = 1;
		if (accel.getX() < 0) multiplier = -1;
		double baselinedVal = getRawAccelForwardDirectionVal() - accelBaseline;
		if (Math.abs(baselinedVal) < 0.03) {
			return 0;
		}
		return multiplier * baselinedVal * 9.8;
	}
	
	public void goStraight(double power){
		smoothDrive(power, power);				
	}
	
	

	public boolean isTimeUp(){
		return (timer.get() >= targetTime);
	}
	

	public void moveWithJoysticks() {
		double rawLeftAxis = -1 * Robot.oi.gamepad.getRawAxis(RobotMap.LEFT_GAMEPAD_JOYSTICK_Y);
		double rawRightAxis = -1 * Robot.oi.gamepad.getRawAxis(RobotMap.RIGHT_GAMEPAD_JOYSTICK_Y);
		
		double leftPower = rawLeftAxis * RobotMap.robotSpeed;
		double rightPower = rawRightAxis * RobotMap.robotSpeed;
		
		
//		if (rawLeftAxis == 0 && rawRightAxis == 0) stop();
//		else {
//			driveCalibratedSquared(rawLeftAxis, rawRightAxis);
//		}
//		
		driveCalibratedSquared(leftPower, rightPower);
//		SmartDashboard.putNumber("Accel Val", getAccelForwardDirectionVal());

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


	public void prepareToTurn() {
		gyro.reset();
		mode = DriveTrainMode.ROTATE;
		stop();
	}
	
	public void turnGiven(double angle) {
		prepareToTurn();
		this.setSetpoint(90);
    	this.setAbsoluteTolerance(10);
    	this.enable();
	}

	public void prepareManual() {
		mode = DriveTrainMode.MANUAL;
	}


	
    
    
}

