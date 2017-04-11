package org.usfirst.frc.team4910.robot.subsystems;

import java.util.ArrayDeque;
import java.util.Queue;

import org.usfirst.frc.team4910.robot.OI;
import org.usfirst.frc.team4910.robot.RobotMap;

import edu.wpi.first.wpilibj.Timer;

public class DriveControllerTurnInPlace {
	private static double kP = RobotMap.DriveTurnPID.KpFast;
	private static double kI = RobotMap.DriveTurnPID.KiFast;
	private static double kD = RobotMap.DriveTurnPID.KdFast;
	
	public static double goalHeading;
	public static double lastError;
	public static double sumError;
	public static boolean reset=true;
	public static double lastTimestamp;
	public static double maxValue=.75;
	public static double minSpeed=.24; //flexible
	public static double absoluteMinimum=.04;
	private static double error;
	private static double errorTolerance=1;
	private static int bufSize=512;
	private static double errorTotal=0.0;
	private static Queue<Double> errorBuf = new ArrayDeque<Double>(bufSize+1);;
	
	public static double calculate(boolean fast){
        double dt = RobotMap.dt;
        if (!reset) {
            double now = Timer.getFPGATimestamp();
            dt = now - lastTimestamp;
            lastTimestamp = now;
        } else {
            lastTimestamp = Timer.getFPGATimestamp();
        }
		error = goalHeading-RobotMap.g.getAngle();
        if (reset) {
            // Prevent jump in derivative term when we have been reset.
            reset = false;
            lastError = error;
            sumError = 0;
        }
        double output = kP*error + kD*((error-lastError)/dt);
        if(output>-1.0 && output<1.0){
        	sumError+=error*dt;
        }
        output+=kI*sumError;
        lastError=error;
        
        errorBuf.add(error);
        errorTotal += error;
        // Remove old elements when the buffer is full.
        if (errorBuf.size() > bufSize) {
          errorTotal -= errorBuf.remove();
        }
        output = (Math.abs(output)/output)*Math.min(maxValue, Math.abs(output));
        output = Math.abs(output)<minSpeed ? (Math.abs(output)/output)*minSpeed : output;
        //System.out.println(output);
		return output;
	}	
	/**This also resets the gyro heading*/
	public static void setGoalHeading(double h){
		goalHeading=h;
	}
	public static double getGoalHeading(){
		return goalHeading;

	}
	public static boolean isFinished(){
		//boolean flag = false;
		//if(Math.abs(RobotMap.g.getAngle() - goalHeading) < 1 && Math.abs(RobotMap.LD1.get())+Math.abs(RobotMap.RD1.get()) < .05) flag=true;
		//Timer.delay(.4);
        //return OI.armStick.getButton(9) || (flag && Math.abs(RobotMap.g.getAngle() - goalHeading) < 1 && Math.abs(RobotMap.LD1.get())+Math.abs(RobotMap.RD1.get()) < .02);
		//System.out.println(" "+getAvgError());
		return error<1.1*errorTolerance && errorBuf.size()!=0 && Math.abs(getAvgError())<errorTolerance && Math.abs(RobotMap.LD1.get())+Math.abs(RobotMap.RD1.get())<=2*minSpeed;
	}
	public static void setTolerance(double value){
		errorTolerance=value;
	}
	public static void reset(){
		reset=true;
		sumError=0;
		RobotMap.g.reset();
	}
	private static double getAvgError(){
	    double avgError = 0;
	    // Don't divide by zero.
	    if (errorBuf.size() != 0) avgError = errorTotal / errorBuf.size();
	    return avgError;
	}
	/**I'll get rid of the static variables later so I don't have this insane next method*/
	public static void destructor(){
		goalHeading=0;
		lastError=0;
		sumError=0;
		reset=true;
		lastTimestamp=0;
		
		errorTotal=0.0;
		errorBuf = new ArrayDeque<Double>(bufSize+1);;
	}
}
