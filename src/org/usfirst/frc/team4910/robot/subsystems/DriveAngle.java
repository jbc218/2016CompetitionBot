package org.usfirst.frc.team4910.robot.subsystems;

import org.usfirst.frc.team4910.robot.RobotMap;

import edu.wpi.first.wpilibj.command.PIDSubsystem;
@Deprecated
public class DriveAngle extends PIDSubsystem {

	public DriveAngle(){
		super("DriveAngle",.1,0,0);
		getPIDController().setContinuous(false);
	}
	@Override
	protected double returnPIDInput() {
		return RobotMap.g.getAngle();
	}

	@Override
	protected void usePIDOutput(double output) {
		RobotMap.RD1.set(output);
		RobotMap.LD1.set(output);
	}

	@Override
	protected void initDefaultCommand() {
	}

}
