package org.usfirst.frc.team5905.subsystems;

import org.usfirst.frc.team5905.Robot;
import org.usfirst.frc.team5905.RobotMap;
import org.usfirst.frc.team5905.commands.*;

import edu.wpi.first.wpilibj.SpeedController;
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
    
    public void spin(int direction, double speed) {
		RobotMap.ARM_LEFT_SPEED_CONTROLLER.set(speed * direction);
		RobotMap.ARM_RIGHT_SPEED_CONTROLLER.set(-1 * speed * direction);
	}

	public void spinInward() {
		spin(1, RobotMap.INTAKE_SPEED);	
	}
	
	public void spinOutward() {
		spin(-1, RobotMap.OUTTAKE_SPEED);
	}
	
	public void stopSpin() {
		spin(0, 0);
	}
}

