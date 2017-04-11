package org.usfirst.frc.team4910.robot.commands;
import org.usfirst.frc.team4910.*;
import org.usfirst.frc.team4910.robot.OI;
import org.usfirst.frc.team4910.robot.Robot;
import org.usfirst.frc.team4910.robot.RobotMap;
import org.usfirst.frc.team4910.robot.subsystems.DriveTrain;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
@Deprecated
public class ArmTest extends Command {
	private static boolean isFinished=false;
	private static int currentEnc;
	private static double currentPot;
	private static int currentL;
	private static int currentR;
	private static long t;
	private static long beginningTime;
	public ArmTest(){
		requires(Robot.in);
	}
	@Override
	protected void initialize() {
		isFinished=false;
		beginningTime=System.currentTimeMillis();
		reiterate(-1);
		System.out.println();
//		while(!OI.shootStick.getButton(10)){
//			RobotMap.topArm.set(OI.shootStick.getY());
//        	RobotMap.armBase.set(OI.armStick.getY());
//        	DriveTrain.drive(-OI.leftStick.getY(), OI.rightStick.getY());
//			if(OI.shootStick.getButton(3)){
//				System.out.println();
//				reiterate(System.currentTimeMillis());
//				Timer.delay(.5);
//			}
//		}
		
//		
//		end();
//		isFinished=true;
	}

	@Override
	protected void execute() {
		
	}

	@Override
	protected boolean isFinished() {
		return isFinished;
	}

	@Override
	protected void end() {
		//Robot.in.angle(0);
		isFinished=true;
	}

	@Override
	protected void interrupted() {
		//Robot.in.angle(0);
		isFinished=true;
	}
	private void reiterate(long time){
		if(time==-1){ System.out.println("Start");
			currentEnc = RobotMap.topEnc.get();
			currentPot = RobotMap.basePot.get();
			currentR = RobotMap.RD1.getEncPosition();
			currentL = RobotMap.LD1.getEncPosition();
			System.out.println("E: "+currentEnc+" P: "+currentPot);
			t=System.currentTimeMillis();
			RobotMap.LD1.setEncPosition(0);
			RobotMap.RD1.setEncPosition(0);
		}else if(time>1000){
			System.out.println("Time: "+(int)((time-t)/1000)+" Seconds");
			System.out.println("E: "+(RobotMap.topEnc.get()-currentEnc)+" P: "+(RobotMap.basePot.get()-currentPot));
			System.out.println("R: "+(RobotMap.RD1.getEncPosition()-currentR)+" L: "+(RobotMap.LD1.getEncPosition()-currentL));
			currentEnc = RobotMap.topEnc.get();
			currentPot = RobotMap.basePot.get();
			currentR = RobotMap.RD1.getEncPosition();
			currentL = RobotMap.LD1.getEncPosition();
			t=System.currentTimeMillis();
			
			
		}
		
	}
}
