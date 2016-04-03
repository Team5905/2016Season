package org.usfirst.frc.team5905.subsystems;

import org.usfirst.frc.team5905.RobotMap;
import org.usfirst.frc.team5905.commands.ArmDown;

import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class Arm extends Subsystem {
    
	private final SpeedController arm = RobotMap.ARM_SPEED_CONTROLLER;

    public void initDefaultCommand() {
    	setDefaultCommand(new ArmDown());
    }

	public void moveArmUp() {
		moveArm(-1);
	}
	
	public void moveArmDown() {
		moveArm(1);
	}
	
	public void moveArm(int direction){
		if (direction != 0)
			arm.set(0.5 * direction);
		else 
			arm.set(0.03);
	}
    
    
}

