package org.usfirst.frc.team4910.robot.subsystems;

import org.usfirst.frc.team4910.robot.RobotMap;

import edu.wpi.first.wpilibj.command.PIDSubsystem;
@Deprecated
public class DriveSpeed extends PIDSubsystem {

	public DriveSpeed(){
		super("DriveSpeed",RobotMap.DriveSpeedPID.KpFast,RobotMap.DriveSpeedPID.KiFast,RobotMap.DriveSpeedPID.KdFast);
		setAbsoluteTolerance(256.0);
		getPIDController().setContinuous(false);
	}
	@Override
	protected double returnPIDInput() {
		return RobotMap.LD1.getSpeed(); //left positive, right negative
	}

	@Override
	protected void usePIDOutput(double output) {
		RobotMap.RD1.set(-output);
		RobotMap.LD1.set(output);
	}

	@Override
	protected void initDefaultCommand() {
	}

}
