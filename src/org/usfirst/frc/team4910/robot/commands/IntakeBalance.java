package org.usfirst.frc.team4910.robot.commands;
import org.usfirst.frc.team4910.*;
import org.usfirst.frc.team4910.robot.OI;
import org.usfirst.frc.team4910.robot.Robot;
import org.usfirst.frc.team4910.robot.RobotMap;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
public class IntakeBalance extends Command {
	private static boolean isFinished=false;
    private static double N = RobotMap.magicN;
    private static double E = RobotMap.magicE;
    private static double M = RobotMap.magicM;
	public IntakeBalance(){
		requires(Robot.in);
	}
	@Override
	protected void initialize() {
		isFinished=false;
		RobotMap.INA.set(M);
		Timer.delay(.118);
		//RobotMap.INA.stopMotor();
		RobotMap.INA.set(N);
		//while(!OI.shootStick.getTrigger());
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
		RobotMap.INA.set(0);
		isFinished=true;
	}

	@Override
	protected void interrupted() {
		// TODO Auto-generated method stub
		//RobotMap.INA.set(0);
		RobotMap.INA.set(0);
		isFinished=true;
	}
	
}
