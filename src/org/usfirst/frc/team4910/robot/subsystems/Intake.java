package org.usfirst.frc.team4910.robot.subsystems;
import org.usfirst.frc.team4910.robot.*;

import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Subsystem;

public class Intake extends Subsystem {
	private static Intake instance;
	private static SpeedController INA;
	private static SpeedController INS;
	
	public static Intake getInstance(){
		return instance==null ? new Intake() : instance;
	}
	
	public void angle(double speed){
		INA.set(speed);
		
	}
	public void wheels(double speed){
		INS.set(speed);
	}
	public Intake(){
		INA=RobotMap.INA;
		INS=RobotMap.INS;
	}

	@Override
	protected void initDefaultCommand() {
		// TODO Auto-generated method stub
		
	}
	
}
