package org.usfirst.frc.team4910.robot;

import edu.wpi.first.wpilibj.*;
import com.ctre.CANTalon;
import com.ctre.CANTalon.TalonControlMode;

import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;

import java.util.ArrayList;
import java.util.Scanner;
import java.util.Vector;


public class RobotMap {
	/**
	 * It just makes things simpler than using an array, and it's what we should do next year.
	 * It's still complicated, but at least it has some functionality.
	 */
	public static class PIDConstants{
		public double KpSlow, KiSlow, KdSlow, KpFast, KiFast, KdFast;
		public PIDConstants(double KpSlow, double KiSlow, double KdSlow, double KpFast, double KiFast, double KdFast){
			this.KpSlow=KpSlow;
			this.KiSlow=KiSlow;
			this.KdSlow=KdSlow;
			this.KpFast=KpFast;
			this.KiFast=KiFast;
			this.KdFast=KdFast;
		}
		public PIDConstants(double Kp, double Ki, double Kd){
			this.KpSlow=Kp;
			this.KiSlow=Ki;
			this.KdSlow=Kd;
			this.KpFast=Kp;
			this.KiFast=Ki;
			this.KdFast=Kd;
		}
		
	}
	public static CANTalon LD1; //left drive 1 and 2. There's 2 motors, and one motor controller can't handle both.
	public static CANTalon LD2;
	public static CANTalon RD1;
	public static CANTalon RD2;
	public static CANTalon INS; //Intake "shooter" or the wheels under it
	public static CANTalon shoot; //Actual shooter
	public static Victor armBase; //Bottom arm
	public static Victor topArm; //Top arm
	public static Encoder topEnc; //Encoder for the top arm
	public static AnalogPotentiometer basePot; //Potentiometer for the bottom arm, measures angle
	public static CANTalon INA; //Intake "angle" or what makes the shield move up and down
	public static AnalogGyro g; //Gyro
    public static PIDConstants PIDLP; //left pos
    public static PIDConstants PIDRP; //right pos
    public static PIDConstants PIDLS; //left speed
    public static PIDConstants PIDRS; //right speed
    public static PIDConstants PIDAT; //arm top pos
    public static PIDConstants PIDAB; //arm base pos
    public static PIDConstants DriveTurnPID = new PIDConstants(.01, 0.0, 0.0,0.15,0.0000,0.0); //fast to be used with camera
    public static PIDConstants DriveDistPID = new PIDConstants(1.0,0.0,0.0); //untested
    public static PIDConstants DriveSpeedPID = new PIDConstants(0.5,0.0,0.3); //tested, but not really
	public static final double magicN=.31; //magic number for normal power to intake (deprecated)
	public static final double magicE=.34; //magic number for extra power to intake (deprecated)
	public static final double magicM=.44; //magic number for moving the intake (deprecated)
	public static PIDController ABPID;
	public static PIDController ATPID;
	public static double dt = 0.005; //Average amount of time for one cycle (unused)
	//public static Encoder eLeft;
	//public static Encoder eRight;
	//public static final double WheelDiameter=1.0/3.0; //try to measure in feet (complicated, unused)
	public static void init(){
		LD1 = new CANTalon(3); //3 //these comments were notes from a while ago
		LD2 = new CANTalon(4); //4
		RD1 = new CANTalon(1); //14
		RD2 = new CANTalon(2); //2
		LD2.changeControlMode(TalonControlMode.Follower); //this puts them in "slave" mode, where it just copies the final input of the motors
		RD2.changeControlMode(TalonControlMode.Follower); //notice I said final input, so if you use a PID loop that CANTalon can control, it will just use the final output
		//speed pid values .5, 0, .3 //old comments
		//distance pid values 1.6, 0, 0
		LD2.set(3); //set to the port of the motor you want followed, if you accidently did LD2.set(OI.leftStick.getY()); then it would freak out (assuming it's still in follower mode)
		RD2.set(1);
		INS = new CANTalon(5); //5
		INA = new CANTalon(7); //7
		shoot = new CANTalon(6); //6
		armBase = new Victor(1); //1
		topArm = new Victor(2); //2
		basePot = new AnalogPotentiometer(1,3600,-2820); //for practice bot: 3600 -1566
		topEnc = new Encoder(0,1,true,EncodingType.k4X); //the kNX depends on what you're using it for, I'll go over that when we explain encoders.
		INA.changeControlMode(TalonControlMode.PercentVbus);
    	PIDLP = new PIDConstants(1.0,0.0,0.0); //1,0,0 just means untested in these.
    	PIDRP = new PIDConstants(1.0,0.0,0.0);
    	PIDLS = new PIDConstants(1.0,0.0,0.0);
    	PIDRS = new PIDConstants(1.0,0.0,0.0);
    	PIDAT = new PIDConstants(0.005,0.000001,0.0); //I don't think I even used these.
    	PIDAB = new PIDConstants(.01,0.0,0.0);
    	g = new AnalogGyro(0);
    	g.initGyro(); //these should be self explanatory
    	g.calibrate();
		
		
		
	}
	
	
}
