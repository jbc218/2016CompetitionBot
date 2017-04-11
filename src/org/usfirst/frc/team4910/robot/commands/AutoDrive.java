package org.usfirst.frc.team4910.robot.commands;

import org.usfirst.frc.team4910.robot.RobotMap;
import org.usfirst.frc.team4910.robot.subsystems.DriveTrain;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
@Deprecated
public class AutoDrive extends CommandGroup {
    
	/**
	 * 
	 * @param currentPos 1-5 on auto line, left-right
	 * @param endingPos 1-3 to castle, left-mid-right
	 * @param defense "lowbar" "port" "draw" "sally" "moat" "rough" "cdf" "rampart" "rockwall" We may or may not need these
	 */
    public AutoDrive(int currentPos, int endingPos, String defense) {
    	if(currentPos==1){
    		
    	}else if(currentPos==2){
    		
    	}else if(currentPos==3){
    		
    	}else if(currentPos==4){
    		
    	}else if(currentPos==5){
    		
    	}
    	
    	
        // Add Commands here:
        // e.g. addSequential(new Command1());
        //      addSequential(new Command2());
        // these will run in order.

        // To run multiple commands at the same time,
        // use addParallel()
        // e.g. addParallel(new Command1());
        //      addSequential(new Command2());
        // Command1 and Command2 will run in parallel.

        // A command group will require all of the subsystems that each member
        // would require.
        // e.g. if Command1 requires chassis, and Command2 requires arm,
        // a CommandGroup containing them would require both the chassis and the
        // arm.
    }
}
