package org.usfirst.frc.team4910.robot.commands;

import org.usfirst.frc.team4910.robot.OI;
import org.usfirst.frc.team4910.robot.Robot;
import org.usfirst.frc.team4910.robot.RobotMap;
import org.usfirst.frc.team4910.robot.subsystems.DriveTrain;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

public class LowGoal extends Command {
	private static boolean isFinished;
    private static double N = RobotMap.magicN;
    private static double E = RobotMap.magicE;
    private static double M = RobotMap.magicM;
	public LowGoal(){
		requires(Robot.in);
	}
	@Override
	protected void initialize() {
		isFinished=false;
		DriveTrain.drive(-OI.leftStick.getY(), OI.rightStick.getY());
		//RobotMap.INA.set(.73);
		//Timer.delay(.7);
		//RobotMap.INA.set(.1);
		RobotMap.INS.set(.2);
		Timer.delay(.4);
		RobotMap.INS.set(0);
		RobotMap.INA.set(.34); //.68
		Timer.delay(.7);
		RobotMap.INS.set(1);
		RobotMap.INA.set(.38);
		Timer.delay(.5);
		RobotMap.INA.set(0);
		end();
		isFinished=true;
	}
	
	@Override
	protected void execute() {
		// TODO Auto-generated method stub
		
	}

	@Override
	protected boolean isFinished() {
		// TODO Auto-generated method stub
		return isFinished;
	}

	@Override
	protected void end() {
		// TODO Auto-generated method stub
		RobotMap.INS.set(0);
		isFinished=true;
	}

	@Override
	protected void interrupted() {
		// TODO Auto-generated method stub
		RobotMap.INS.set(0);
		isFinished=true;
	}
}
