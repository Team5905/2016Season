package org.usfirst.frc.team5905.subsystems;

import org.usfirst.frc.team5905.RobotMap;

public class RobotMath {

	public static double ease(double target, double previous, double easeIncrement) {
		double newVal;
		if (target < previous - easeIncrement) {
			newVal = previous - easeIncrement;
		} else if (target > previous + easeIncrement) {
			newVal = previous + easeIncrement;
		} else {
			newVal = target;
		}

		return newVal;
	}

	public static double ease(double target, double previous) {
		return ease(target, previous, RobotMap.EASE_INCREMENT);
	}
}
