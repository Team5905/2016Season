package org.usfirst.frc.team5905.subsystems;

import org.usfirst.frc.team5905.Robot;
import org.usfirst.frc.team5905.RobotMap;
import org.usfirst.frc.team5905.commands.*;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class Intake extends Subsystem {
    
    // Put methods for controlling this subsystem
    // here. Call these from Commands.

    public void initDefaultCommand() {
        setDefaultCommand(new SpinInward());
    }
    
    public void spin(int direction) {
		RobotMap.ARM_LEFT_SPEED_CONTROLLER.set(RobotMap.INTAKE_SPEED * direction);
		RobotMap.ARM_RIGHT_SPEED_CONTROLLER.set(-1 * RobotMap.INTAKE_SPEED * direction);
	}

	public void spinInward() {
		spin(1);	
	}
	
	public void spinOutward() {
		spin(-1);
	}
	
	public void stopSpin() {
		spin(0);
	}
}

