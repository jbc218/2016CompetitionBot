package org.usfirst.frc.team4910.robot.commands;
import org.usfirst.frc.team4910.*;
import org.usfirst.frc.team4910.robot.Robot;
import org.usfirst.frc.team4910.robot.RobotMap;
import org.usfirst.frc.team4910.robot.subsystems.DriveTrain;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
public class Shoot extends Command {
    private static double N = RobotMap.magicN;
    private static double E = RobotMap.magicE;
    private static double M = RobotMap.magicM;
	private static boolean isFinished=false;
	public Shoot(){
		requires(Robot.s);
		requires(Robot.in);
	}
	@Override
	protected void initialize() {
		isFinished=false;
		//DriveTrain.drive(Robot.leftGain, Robot.rightGain);
		Robot.in.wheels(.425);
		RobotMap.INA.set(-.1);
		Timer.delay(.25);
		//RobotMap.INA.set(E);
		Robot.in.wheels(0);
		Robot.s.Shoot(1);
		Timer.delay(1.5); //changed
		Robot.in.wheels(-1);
		Timer.delay(1);
		RobotMap.INA.set(0);
		Robot.s.Shoot(0);
		Robot.in.wheels(0);
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
		
		Robot.s.Shoot(0);
		Robot.in.wheels(0);
		isFinished=true;
	}

	@Override
	protected void interrupted() {
		// TODO Auto-generated method stub
		Robot.s.Shoot(0);
		Robot.in.wheels(0);
		isFinished=true;
	}
	
}
