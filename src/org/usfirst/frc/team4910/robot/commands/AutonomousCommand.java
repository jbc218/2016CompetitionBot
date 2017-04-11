package org.usfirst.frc.team4910.robot.commands;

import org.usfirst.frc.team4910.robot.Robot;
import org.usfirst.frc.team4910.robot.subsystems.DriveTrain;

import edu.wpi.first.wpilibj.command.Command;

/**
 *Although I hate to deprecate a command as important as this, it isn't used.
 */
@Deprecated
public class AutonomousCommand extends Command {
	String autoPos;
    public AutonomousCommand() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires(Robot.driveTrain);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	//autoPos = (String) Robot.autoChoose.getSelected();
    	
    	
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
