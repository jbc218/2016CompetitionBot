package org.usfirst.frc.team4910.robot.commands;

import org.usfirst.frc.team4910.*;
import org.usfirst.frc.team4910.robot.OI;
import org.usfirst.frc.team4910.robot.Robot;
import org.usfirst.frc.team4910.robot.RobotMap;
import org.usfirst.frc.team4910.robot.subsystems.DriveTrain;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

public class Collect extends Command {
    private static double N = RobotMap.magicN;
    private static double E = RobotMap.magicE;
    private static double M = RobotMap.magicM;
	private static boolean isFinished=false;
	public Collect(){
		super();
		requires(Robot.in);
		requires(Robot.driveTrain);
	}
	boolean b;
	@Override
	protected void initialize() {
		isFinished=false;
		b=false;
		Robot.collectEnd=false;
		//RobotMap.INA.set(M);
		//Timer.delay(.125);
		//RobotMap.INA.set(E);
		
		RobotMap.INS.set(-.5);
		RobotMap.INA.set(0);
		RobotMap.shoot.set(-.6);
		Timer.delay(.2);
		while(!OI.shootStick.getTrigger()){
			RobotMap.shoot.set(-.65);
			if(OI.rightStick.getButton(3)) {
				RobotMap.INA.set(.75);
			}else{
				RobotMap.INA.set(0);
			}
			 RobotMap.LD1.set(-OI.leftStick.getY());
		     //RobotMap.LD2.set(-OI.leftStick.getY());
		     RobotMap.RD1.set(OI.rightStick.getY());
		     //RobotMap.RD2.set(OI.rightStick.getY());
			//RobotMap.INA.set(N+(OI.shootStick.getY()/50));
			while(OI.rightStick.getButton(4)){
				RobotMap.INA.set(-.5);
				b=true;
			}
			if(b) RobotMap.INA.set(0);
		}
		RobotMap.shoot.set(0);
		/*RobotMap.INA.set(0.1496);
		Robot.driveTrain.drive(0);
		Timer.delay(0.57);
		RobotMap.INA.set(.1);
		Timer.delay(.05);
		RobotMap.INA.set(0);
		Robot.driveTrain.drive(0);
		Timer.delay(.1);
		RobotMap.INA.set(-0.1);
		Timer.delay(.3);
		RobotMap.INS.set(0.0);
		if(OI.shootStick.getButton(3)){
			RobotMap.shoot.set(.2);
			RobotMap.INS.set(-.25);
			Timer.delay(.1);
		}
		RobotMap.shoot.set(0);*/
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
		//RobotMap.INA.set(0);
		RobotMap.INS.set(0);
		//Robot.driveTrain.drive(0);
		RobotMap.INA.set(0);
		RobotMap.INA.set(0);
		Robot.collectEnd=true;
		isFinished=true;
	}

	@Override
	protected void interrupted() {
		// TODO Auto-generated method stub
		//RobotMap.INA.set(0);
		RobotMap.INS.set(0);
		//Robot.driveTrain.drive(0);
		RobotMap.INA.set(0);
		Robot.collectEnd=true;
		isFinished=true;
	}
}
