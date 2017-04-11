package org.usfirst.frc.team4910.robot.commands;

import org.usfirst.frc.team4910.robot.*;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
@Deprecated
public class Testing extends Command {
    private static double N = RobotMap.magicN;
    private static double E = RobotMap.magicE;
    private static double M = RobotMap.magicM;
	private static boolean isFinished=false;
	public Testing(){
		requires(Robot.in);
	}
	@Override
	protected void initialize() {
		isFinished=false;
        double kP1=RobotMap.PIDAT.KpFast;
        double kI1=RobotMap.PIDAT.KiFast;
        double kD1=RobotMap.PIDAT.KdFast;
        double kP2=RobotMap.PIDAB.KpFast;
        double kI2=RobotMap.PIDAB.KiFast;
        double kD2=RobotMap.PIDAB.KdFast;
        //System.out.println("t "+RobotMap.topEnc.get());
        //System.out.println("b "+RobotMap.basePot.get());
        //System.out.println("kpt "+kP1+" kIt "+kI1+" kdt "+kD1);
        //System.out.println("kpb "+kP2+" kIb "+kI2+" kdb "+kD2);
    	RobotMap.ATPID = new PIDController(kP1, kI1, kD1, RobotMap.topEnc, RobotMap.topArm);
		RobotMap.ABPID = new PIDController(kP2, kI2, kD2, RobotMap.basePot, RobotMap.armBase);
		RobotMap.ATPID.enable();
		RobotMap.ABPID.enable();
		RobotMap.ABPID.setSetpoint(150);
		RobotMap.ATPID.setSetpoint(0);
		System.out.println("Test");
		//Timer.delay(1);
		end();
		Robot.hasIterated=true;
		isFinished=true;
	}
	
	@Override
	protected void execute() {
    	System.out.println("kptN "+RobotMap.ATPID.getP()+" kItN "+RobotMap.ATPID.getI()+" kdtN "+RobotMap.ATPID.getD());
        System.out.println("kpbN "+RobotMap.ABPID.getP()+" kIbN "+RobotMap.ABPID.getI()+" kdbN "+RobotMap.ABPID.getD());
//		if(OI.shootStick.getButton(1)){
//			end();
//		}
//		
		//12 = 0 deg
		//1 = -7 deg
		//81 = 90 deg
		//141 = 173 deg
		
		//83 deg / 60 w  d=(83/60)w+16.6
	}

	@Override
	protected boolean isFinished() {
		// TODO Auto-generated method stub
		return isFinished;
	}

	@Override
	protected void end() {
		// TODO Auto-generated method stub
		//Robot.in.angle(0);
		isFinished=true;
	}

	@Override
	protected void interrupted() {
		// TODO Auto-generated method stub
		//Robot.in.angle(0);
		isFinished=true;
	}
	
}
