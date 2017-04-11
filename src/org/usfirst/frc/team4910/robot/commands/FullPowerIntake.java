package org.usfirst.frc.team4910.robot.commands;

import org.usfirst.frc.team4910.robot.OI;
import org.usfirst.frc.team4910.robot.Robot;
import org.usfirst.frc.team4910.robot.RobotMap;
import org.usfirst.frc.team4910.robot.subsystems.DriveTrain;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class FullPowerIntake extends Command {
	public static boolean isFinished=true;
	private static double E = RobotMap.magicE;
	private static double N = RobotMap.magicN;
    public FullPowerIntake() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires(Robot.in);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
//    	while(OI.rightStick.getButton(3)) {
//    		RobotMap.INA.set(1);
//    	}
//		
//		RobotMap.INA.set(N);
//		end();
		isFinished=false;
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
		isFinished=false;
    	RobotMap.INA.set(1);
//    	DriveTrain.drive(-OI.leftStick.getY(), OI.rightStick.getY());
//    	if(!OI.rightStick.getButton(3)){isFinished=true; end();}
    	
    	
    	
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return isFinished;
    }

    // Called once after isFinished returns true
    protected void end() {
    	RobotMap.INA.set(0);
		isFinished=true;
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	RobotMap.INA.set(0);
    	isFinished=true;
    }
}
