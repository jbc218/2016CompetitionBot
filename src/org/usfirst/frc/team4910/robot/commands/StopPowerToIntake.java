package org.usfirst.frc.team4910.robot.commands;

import org.usfirst.frc.team4910.robot.*;
import org.usfirst.frc.team4910.robot.subsystems.DriveTrain;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
public class StopPowerToIntake extends Command {
    private static double N = RobotMap.magicN;
    private static double E = RobotMap.magicE;
    private static double M = RobotMap.magicM;
	private static boolean isFinished=false;
	public StopPowerToIntake(){
		requires(Robot.in);
	}
	@Override
	protected void initialize() {
		isFinished=false;
		
		while(OI.rightStick.getButton(5)){ RobotMap.INA.set(0); DriveTrain.drive(-OI.leftStick.getY(), OI.rightStick.getY());}
		Robot.hasIterated=true;
		isFinished=true;
	}
	
	@Override
	protected void execute() {
		
	}

	@Override
	protected boolean isFinished() {
		// TODO Auto-generated method stub
		return isFinished;
	}

	@Override
	protected void end() {
		// TODO Auto-generated method stub
		RobotMap.INA.set(0);
		isFinished=true;
	}

	@Override
	protected void interrupted() {
		// TODO Auto-generated method stub
		RobotMap.INA.set(0);
		isFinished=true;
	}
	
}
