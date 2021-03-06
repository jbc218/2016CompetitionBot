// RobotBuilder Version: 0.0.2
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in th future.


package org.usfirst.frc.team4910.robot.subsystems;

import org.usfirst.frc.team4910.robot.RobotMap;
import org.usfirst.frc.team4910.util.Units;
import org.usfirst.frc.team4910.robot.OI;
import org.usfirst.frc.team4910.robot.Robot;

import edu.wpi.first.wpilibj.*;
import com.ctre.CANTalon;
import com.ctre.CANTalon.TalonControlMode;
import edu.wpi.first.wpilibj.PIDController.Tolerance;
import edu.wpi.first.wpilibj.command.PIDSubsystem;
import edu.wpi.first.wpilibj.command.Subsystem;


/**
 *
 */
@SuppressWarnings("unused")
public class DriveTrain extends Subsystem {
    public static final double threshold=0.15;
    private static DriveTrain instance = null;
    private static SpeedController left1 = RobotMap.LD1;
    private static SpeedController right1 = RobotMap.RD1;
    private static SpeedController left2 = RobotMap.LD2;
    private static SpeedController right2 = RobotMap.RD2;
    private static double N = RobotMap.magicN;
    private static double E = RobotMap.magicE;
    private static double M = RobotMap.magicM;
    //private static Encoder eL = RobotMap.eLeft;
    //private static Encoder eR = RobotMap.eRight;
    public void initDefaultCommand() {
    }
    public static DriveTrain getInstance() {
		if (instance == null) {
			instance = new DriveTrain();
		}
		return instance;
	}
    private DriveTrain(){}

    private static boolean switched=false;
    public static void switchDrive(){
//    	if(!switched){
//    		while(OI.leftStick.getButton(2) || OI.rightStick.getButton(2)){
//    			switched=true;
//    		}
//    	}else{
//    		while(OI.leftStick.getButton(2) || OI.rightStick.getButton(2)){
//    			switched=false;
//    		}
//    	}
    	
    }
    public static void drive(double leftSpeed, double rightSpeed){ //again, if one should be negative, switch it down here
        if ( Math.abs(leftSpeed) < threshold) {
            left1.set(0);
            //left2.set(0);
        }else{
        	if(!switched){
        		left1.set(leftSpeed);
            	//left2.set(leftSpeed);
        	}else{
        		right1.set(-leftSpeed);
            	//right2.set(-leftSpeed);
        	}
        }
        if ( Math.abs(rightSpeed) < threshold ) {
            right1.set(0);
            //right2.set(0);
        }else{
        	if(!switched){
        		right1.set(rightSpeed);
            	//right2.set(rightSpeed);
        	}else{
        		left1.set(-rightSpeed);
            	//left2.set(-rightSpeed);
        	}
        }
    }
    /** No thresh*/
    public static void driveNT(double left, double right){
    	left1.set(left);
    	right1.set(right);
    }
    /** Shouldn't be used, but it's here anyway */
    public void drive(double speed){
    	drive(-speed, speed);
    }
    /**
     * Deprecated code, don't use it
     * It's only here because it was used in autonomous once, pretend we used magic at GRITS instead
     * 
     * @param theta
     * @param distance
     */
    public static void driveAngleAndDistance(double theta, double distance){
    	RobotMap.LD1.setEncPosition(0);
    	RobotMap.RD1.setEncPosition(0);
    	TalonControlMode Rc = RobotMap.RD1.getControlMode(), Lc = RobotMap.LD1.getControlMode();
    	RobotMap.RD1.changeControlMode(TalonControlMode.PercentVbus);
    	RobotMap.LD1.changeControlMode(TalonControlMode.PercentVbus);
    	PIDController gR = new PIDController(0.05,0,0.0,RobotMap.g,RobotMap.RD1);
    	//TODO: put this in robotmap rather than here

    	PIDController gL = new PIDController(0.05,0,0.0,RobotMap.g,RobotMap.LD1);
    	
    	gR.setSetpoint(RobotMap.g.getAngle()+theta);
    	gL.setSetpoint(RobotMap.g.getAngle()+theta);
    	gR.enable();
    	gL.enable();
    	gL.setAbsoluteTolerance(3);
    	gR.setAbsoluteTolerance(3);
    	gR.setSetpoint(RobotMap.g.getAngle()+theta); //just to make sure
    	gL.setSetpoint(RobotMap.g.getAngle()+theta);
    	
    	/*while(gR.onTarget()){ //use if above doesn't work
    		RobotMap.LD1.set(-RobotMap.RD1.getOutputVoltage()/RobotMap.RD1.getBusVoltage());
    	}*/
    	if(theta>0)Timer.delay(1.2);
    	gR.disable();
    	gL.disable();
    	RobotMap.RD1.changeControlMode(TalonControlMode.Position);
    	RobotMap.LD1.changeControlMode(TalonControlMode.Position);
    	RobotMap.RD1.setPID(RobotMap.PIDRP.KpFast, RobotMap.PIDRP.KiFast, RobotMap.PIDRP.KdFast);
    	RobotMap.LD1.setPID(RobotMap.PIDLP.KpFast, RobotMap.PIDLP.KiFast, RobotMap.PIDLP.KdFast);
    	RobotMap.LD1.setEncPosition(0);
    	RobotMap.RD1.setEncPosition(0);
    	RobotMap.LD1.set(-Units.DTC(distance));
    	RobotMap.RD1.set(Units.DTC(distance));
    	System.out.println("\n\nSetpoint L: "+Units.DTC(distance)+"\nPosL: "+RobotMap.LD1.getPosition());
    	System.out.println("\n\nSetpoint R: "+Units.DTC(distance)+"\nPosR: "+RobotMap.RD1.getPosition());
    	//I have no idea how long I should let this run, since it depends on error. I'll let the threshhold be 100
    	
		System.out.println("LE: "+RobotMap.LD1.getError());
		System.out.println("RE: "+RobotMap.RD1.getError());
		Long asdf = System.currentTimeMillis()/1000L;
    	while(((Math.abs(RobotMap.LD1.getError())+Math.abs(RobotMap.RD1.getError()))/2.0>250)&&!Robot.currentStatus.equals(Robot.RobotStatus.DISABLED)){
    		System.out.println("LE: "+RobotMap.LD1.getError());
    		System.out.println("RE: "+RobotMap.RD1.getError());
    		if(((System.currentTimeMillis()/1000L)-asdf>3.5) && ((dVdt(RobotMap.RD1)<0 && dVdt(RobotMap.LD1)<0) && (RobotMap.LD1.getOutputVoltage()<1 || RobotMap.RD1.getOutputVoltage()<1) ) ) break;
        	System.out.println("\n\nSetpoint L: "+Units.DTC(distance)+"\nPosL: "+RobotMap.LD1.getPosition());
        	System.out.println("\n\nSetpoint R: "+Units.DTC(distance)+"\nPosR: "+RobotMap.RD1.getPosition());
    	}
    	
    	RobotMap.LD1.setPID(0, 0, 0);
    	RobotMap.RD1.setPID(0, 0, 0);
    	RobotMap.LD1.changeControlMode(Lc);
    	RobotMap.RD1.changeControlMode(Rc);
    	//TODO: store pid values at beginning and give back at end of loop
    	
    }
    /**
     * Useless, but returns the rate that voltage is changing over time.
     * Could be improved by running in a separate thread
     * 
     * @return rate of change
     */
    private static double dVdt(CANTalon t){
    	double v1 = t.getOutputVoltage();
    	Timer.delay(.08);
    	double v2 = t.getOutputVoltage();
    	return (v2-v1)/(.01);
    }

    public static void driveToDistance(double dist){
    	Robot.driveDistance.enable();
    	Robot.driveDistance.setSetpoint(dist);
    	while(!Robot.currentStatus.equals(Robot.RobotStatus.DISABLED) && !Robot.driveDistance.onTarget()) System.out.println(dist-Robot.driveDistance.getPosition());
    	Robot.driveDistance.disable();
    }
    /**
     * 
     * Everything below these comments are either untested or unreliable code, you can look through them if you want
     * but please don't take anything here as seriously, as it is experimental and probably wrong (but there are no other options).
     */
    
    public static void driveToAngle(double theta){
    	RobotMap.g.reset();
    	Robot.driveAngle.setAbsoluteTolerance(8.0);
    	Robot.driveAngle.getPIDController().setContinuous(false);
    	Robot.driveAngle.getPIDController().setPID(RobotMap.DriveTurnPID.KpSlow,RobotMap.DriveTurnPID.KiSlow,RobotMap.DriveTurnPID.KdSlow);
    	Robot.driveAngle.enable();
    	Robot.driveAngle.setSetpoint(theta);
    	while(!Robot.currentStatus.equals(Robot.RobotStatus.DISABLED) && !Robot.driveAngle.onTarget()) System.out.println(theta-RobotMap.g.getAngle());
    	Robot.driveAngle.disable();
    	Robot.driveAngle.getPIDController().reset();
    }
    public static void driveToSmallAngle(double theta){
    	isTurning=true;
    	isFinished=false;
    	RobotMap.g.reset();
    	Robot.driveAngle.getPIDController().reset();
    	
    	Robot.driveAngle.setAbsoluteTolerance(1.0);
    	Robot.driveAngle.getPIDController().setContinuous(false);
    	
    	Robot.driveAngle.getPIDController().setPID(RobotMap.DriveTurnPID.KpFast,RobotMap.DriveTurnPID.KiFast,RobotMap.DriveTurnPID.KdFast);
    	Robot.driveAngle.enable();
    	Robot.driveAngle.setSetpoint(theta);
    	/*while(!(Robot.currentStatus.equals(Robot.RobotStatus.DISABLED) || Robot.driveAngle.onTarget())){
    		System.out.println(theta-RobotMap.g.getAngle());
    		Timer.delay(.1);
    	}
    	Robot.driveAngle.getPIDController().reset();
       */
    }
    public static void driveToDistanceCAN(double dist){
    	TalonControlMode Tl = RobotMap.LD1.getControlMode();
    	TalonControlMode Tr = RobotMap.RD1.getControlMode();
    	RobotMap.LD1.changeControlMode(TalonControlMode.Position);
    	RobotMap.RD1.changeControlMode(TalonControlMode.Position);
    	RobotMap.LD1.setEncPosition(0);
    	RobotMap.RD1.setEncPosition(0);
    	RobotMap.LD1.set(-dist);
    	RobotMap.RD1.set(dist);
    	//unfinished
    }
    public static void driveToSpeedCAN(){
    	double left = Math.abs(OI.leftStick.getY())<.1 ? 0 : OI.leftStick.getY();
    	double right = Math.abs(OI.rightStick.getY())<.1 ? 0 : OI.rightStick.getY();
    	left = -1000.0*left;
    	right = 1000.0*right;
    	RobotMap.LD1.set(left);
    	RobotMap.RD1.set(right);
    	//System.out.println("Error Left: "+(-RobotMap.LD1.getEncVelocity()+-(10000.0-10000.0*OI.leftStick.getY())));
    	//System.out.println("Error Right: "+(-RobotMap.RD1.getEncVelocity()+(10000.0-10000.0*OI.rightStick.getY())));
    	System.out.println("left setpoint: "+left);
    	System.out.println("right setpoint: "+right);
    	
    }
    
    private static boolean hasInitAngleS=false,hasInitAngleL=false,hasInitCANDistance=false,hasInitCANSpeed=false,hasInitDistance=false,hasInitSpeed=false,hasInitPVbus=true;
    public static boolean isFinished=true, isTurning=false;
    private static double kOutput=1;
    private static double setpoint=0;
    public enum driveType{
    	PVBUS,
    	GYROPID,
    	CANDISTANCEPID,
    	DISTANCEPID,
    	CANSPEEDPID,
    	SPEEDPID;
    }
    public static driveType currentDriveType=driveType.PVBUS;

    /**
     * This drives according to what buttons were or weren't pressed. Meant to be called repeatedly in teleop
     */
    public static void drive(){
    	if(OI.leftStick.getButton(8)){
    		while(OI.leftStick.getButton(8));
    		destructor();
    		currentDriveType = driveType.PVBUS;
			RobotMap.LD1.changeControlMode(TalonControlMode.PercentVbus);
			RobotMap.RD1.changeControlMode(TalonControlMode.PercentVbus);
	    	RobotMap.LD1.setEncPosition(0);
	    	RobotMap.RD1.setEncPosition(0);
	    	kOutput=1;
	    	hasInitPVbus=true;
    	}
    	if(OI.leftStick.getButton(9)){
    		while(OI.leftStick.getButton(9));
    		destructor();
    		currentDriveType = driveType.CANSPEEDPID; //won't need speedpid this year, just canspeedpid. Might be added for next year
        	RobotMap.LD1.changeControlMode(TalonControlMode.Speed);
        	RobotMap.RD1.changeControlMode(TalonControlMode.Speed);
        	RobotMap.LD1.setEncPosition(0);
        	RobotMap.RD1.setEncPosition(0);
        	RobotMap.RD1.set(0);
        	RobotMap.LD1.set(0);
        	kOutput=1000;
        	hasInitCANSpeed=true;
    	}
    	if(OI.leftStick.getButton(10)){
    		while(OI.leftStick.getButton(10));
    		destructor();
    		currentDriveType = driveType.CANDISTANCEPID;
        	RobotMap.LD1.changeControlMode(TalonControlMode.Position);
        	RobotMap.RD1.changeControlMode(TalonControlMode.Position);
        	RobotMap.LD1.setEncPosition(0);
        	RobotMap.RD1.setEncPosition(0);
        	hasInitCANDistance=true;
    	}
    	if(OI.leftStick.getButton(11)){
    		while(OI.leftStick.getButton(11));
    		destructor();
    		currentDriveType = driveType.GYROPID;
    		RobotMap.LD1.changeControlMode(TalonControlMode.PercentVbus);
    		RobotMap.RD1.changeControlMode(TalonControlMode.PercentVbus);
    		
    	}
    	if(hasInitPVbus || hasInitCANSpeed){
        	double left = Math.abs(OI.leftStick.getY())<.1 ? 0 : OI.leftStick.getY();
        	double right = Math.abs(OI.rightStick.getY())<.1 ? 0 : OI.rightStick.getY();
        	left = -kOutput*left;
        	right = kOutput*right;
        	RobotMap.LD1.set(left);
        	RobotMap.RD1.set(right);
    	}else if(hasInitCANDistance || hasInitCANSpeed){
    		RobotMap.LD1.set(-kOutput*setpoint);
    		RobotMap.RD1.set(kOutput*setpoint);
    	}
    }
    public static void destructor(){
    	if(currentDriveType.equals(driveType.GYROPID)){
			Robot.driveAngle.getPIDController().reset();
			hasInitAngleS=hasInitAngleL=false;
		}else if(currentDriveType.equals(driveType.CANDISTANCEPID) || currentDriveType.equals(driveType.CANSPEEDPID)){
			RobotMap.LD1.changeControlMode(TalonControlMode.PercentVbus);
			RobotMap.RD1.changeControlMode(TalonControlMode.PercentVbus);
	    	RobotMap.LD1.setEncPosition(0);
	    	RobotMap.RD1.setEncPosition(0);
	    	hasInitCANDistance=hasInitCANSpeed=false;
		}else if(currentDriveType.equals(driveType.DISTANCEPID)){
			Robot.driveDistance.getPIDController().reset();
			hasInitDistance=false;
		}else if(currentDriveType.equals(driveType.SPEEDPID)){
			Robot.driveSpeed.getPIDController().reset();
			hasInitSpeed=false;
		}
    }
    public static void driveAngle(double theta){
    	if(!hasInitAngleS && theta<=10){
        	RobotMap.g.reset();
        	Robot.driveAngle.getPIDController().reset();
        	Robot.driveAngle.setAbsoluteTolerance(1.0);
        	Robot.driveAngle.getPIDController().setContinuous(false);
        	Robot.driveAngle.getPIDController().setPID(RobotMap.DriveTurnPID.KpFast,RobotMap.DriveTurnPID.KiFast,RobotMap.DriveTurnPID.KdFast);
        	Robot.driveAngle.enable();
    		hasInitAngleS=true;
    	}
    	if(!hasInitAngleL && theta>10){
        	RobotMap.g.reset();
        	Robot.driveAngle.setAbsoluteTolerance(8.0);
        	Robot.driveAngle.getPIDController().setContinuous(false);
        	Robot.driveAngle.getPIDController().setPID(RobotMap.DriveTurnPID.KpSlow,RobotMap.DriveTurnPID.KiSlow,RobotMap.DriveTurnPID.KdSlow);
        	Robot.driveAngle.enable();
    		hasInitAngleL=true;
    	}
    	if(hasInitAngleL || hasInitAngleS){
    		Robot.driveAngle.setSetpoint(theta);
    	}
    }
    
}

