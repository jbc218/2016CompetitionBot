package org.usfirst.frc.team4910.robot.subsystems;

import org.usfirst.frc.team4910.robot.RobotMap;

import edu.wpi.first.wpilibj.command.PIDSubsystem;
@Deprecated
public class DriveDistance extends PIDSubsystem {

	public DriveDistance(){
		super("DriveDistance",RobotMap.DriveDistPID.KpFast,RobotMap.DriveDistPID.KiFast,RobotMap.DriveDistPID.KdFast);
		setAbsoluteTolerance(20.0);
		getPIDController().setContinuous(false);
	}
	@Override
	protected double returnPIDInput() {
		return RobotMap.LD1.getEncPosition(); //left positive, right negative
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
