package org.usfirst.frc.team4910.robot.commands;

import org.usfirst.frc.team4910.robot.*;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
@Deprecated
public class ArmDrawbridge extends Command {
    private static double N = RobotMap.magicN;
    private static double E = RobotMap.magicE;
    private static double M = RobotMap.magicM;
	private static boolean isFinished=false;
	public ArmDrawbridge(){
		requires(Robot.in);
	}
	@Override
	protected void initialize() {
		isFinished=false;
		Robot.armCommandActive=true;
		double Einit=RobotMap.topEnc.get(); //should be 0
		double Pinit=RobotMap.basePot.get();
		System.out.println(0);
		RobotMap.ATPID.setSetpoint(Einit+268);
		RobotMap.ABPID.setSetpoint(Pinit-16.6);
		Timer.delay(1);
		System.out.println(1);
		RobotMap.ABPID.setSetpoint(RobotMap.ABPID.getSetpoint()-69.1);
		Timer.delay(1);
		System.out.println(2);
		RobotMap.ATPID.setSetpoint(RobotMap.ATPID.getSetpoint()-66);
		RobotMap.ABPID.setSetpoint(RobotMap.ABPID.getSetpoint()+33.9);
		Timer.delay(1);
		System.out.println(3);
		RobotMap.ATPID.setSetpoint(RobotMap.ATPID.getSetpoint()-4);
		RobotMap.ABPID.setSetpoint(RobotMap.ABPID.getSetpoint()+.287);
		Timer.delay(1);
		System.out.println(4);
		RobotMap.ATPID.setSetpoint(RobotMap.ATPID.getSetpoint()-21);
		RobotMap.ABPID.setSetpoint(RobotMap.ABPID.getSetpoint()-6.19);
		Timer.delay(1);
		System.out.println(5);
		RobotMap.ATPID.setSetpoint(RobotMap.ATPID.getSetpoint()-94);
		RobotMap.ABPID.setSetpoint(RobotMap.ABPID.getSetpoint()+30.2);
		Timer.delay(1);
		System.out.println(6);
		RobotMap.ATPID.setSetpoint(RobotMap.ATPID.getSetpoint()-70);
		Timer.delay(1);
		System.out.println(7);
		RobotMap.ATPID.setSetpoint(RobotMap.ATPID.getSetpoint()-0);
		RobotMap.ABPID.setSetpoint(RobotMap.ABPID.getSetpoint()+12);
		Timer.delay(1);
		System.out.println(8);
		RobotMap.ABPID.setSetpoint(RobotMap.ABPID.getSetpoint()-12.9);
		Timer.delay(1);
		System.out.println(9);
		RobotMap.ATPID.setSetpoint(RobotMap.ATPID.getSetpoint()-71);
		RobotMap.ABPID.setSetpoint(RobotMap.ABPID.getSetpoint()+13.6);
		Timer.delay(1);
		System.out.println(10);
		RobotMap.ABPID.setSetpoint(RobotMap.ABPID.getSetpoint()-11.9);
		Timer.delay(1);
		System.out.println(11);
		RobotMap.ATPID.setSetpoint(RobotMap.ATPID.getSetpoint()-124);
		RobotMap.ABPID.setSetpoint(RobotMap.ABPID.getSetpoint()-1.8);
		Timer.delay(1);
		System.out.println(12);
		RobotMap.ATPID.setSetpoint(RobotMap.ATPID.getSetpoint()-164);
		RobotMap.ABPID.setSetpoint(RobotMap.ABPID.getSetpoint()-96.6);
		Timer.delay(1);
		System.out.println(13);
		RobotMap.ATPID.setSetpoint(RobotMap.ATPID.getSetpoint()+322);
		RobotMap.ABPID.setSetpoint(RobotMap.ABPID.getSetpoint()+95.1);
		System.out.println(14);
		Timer.delay(1);
		//RobotMap.ATPID.setSetpoint(RobotMap.ATPID.getSetpoint()-122.33);
		//RobotMap.ABPID.setSetpoint(RobotMap.ABPID.getSetpoint()-335);
		System.out.println("end");
		Timer.delay(1);
		RobotMap.ABPID.setSetpoint(150);
		RobotMap.ATPID.setSetpoint(0);
		/**
		 *  Start 
 E: 0 P: 153.28745467204112 c
  
  
 Time: 42 Seconds 
 E: 268 P: -16.60474297598944  c
 R: -826 L: 743 
  
 Time: 5 Seconds 
 E: 0 P: -69.15177913148113  c
 R: 1 L: 0 
  
 Time: 5 Seconds 
 E: -66 P: 33.89112543538954 c
 R: 0 L: 0 
  
 Time: 5 Seconds 
 E: -4 P: 0.2873798667681058 c
 R: 237 L: -193 
  
 Time: 10 Seconds 
 E: -21 P: -6.192505672623383 c
 R: 368 L: -450 
  
 Time: 11 Seconds 
 E: -94 P: 30.2363948253103 
 R: 202 L: -187 
  
 Time: 10 Seconds 
 E: -76 P: 0.0 
 R: 358 L: -345 
  
 Time: 17 Seconds 
 E: -33 P: 12.039758199414791 
 R: 20 L: -48 
  
 Time: 15 Seconds 
 E: 0 P: -12.95684758697098 
 R: 156 L: -117 
  
 Time: 10 Seconds 
 E: -71 P: 13.64599191161551 
 R: 0 L: -6 
  
 Time: 10 Seconds 
 E: 0 P: -11.918541116768438 
 R: 65 L: -54 
  
 Time: 5 Seconds 
 E: -124 P: -1.7599126149218591 
 R: 69 L: -63 
  
 Time: 6 Seconds 
 E: -164 P: -96.55828939843445 
 R: 79 L: -93 
  
 Time: 4 Seconds 
 E: 322 P: 95.10141523314269 c
 R: -1091 L: 712 

		 */
	
		
		end();
		isFinished=true;
	}
	
	@Override
	protected void execute() {
		//arm base = 13.25 inches
		//arm top (with extra) = 18.25 inches
		//arm top without = 13.75
		//out at start = 4.5 inches
		//base angle = Θ (alt 233)
		//top angle = φ (alt 237)
		//S(Θ,φ) = 4.5+13.25cos(Θ)+18.25cos(φ)
		
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
	
}
