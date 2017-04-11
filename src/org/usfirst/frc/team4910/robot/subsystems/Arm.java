package org.usfirst.frc.team4910.robot.subsystems;

import org.usfirst.frc.team4910.robot.*;


import edu.wpi.first.wpilibj.AnalogInput;
import com.ctre.CANTalon;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.command.Subsystem;
@SuppressWarnings("unused")
@Deprecated
public class Arm extends Subsystem {

	private static Arm instance;
	private static SpeedController armBase;
	private static SpeedController topArm;
	//private static AnalogPotentiometer basePot
	@Override
	protected void initDefaultCommand() {
		// TODO Auto-generated method stub
		
	}
	
	public static Arm getInstance(){
		return instance==null ? new Arm() : instance;
	}
	
	public Arm(){
		armBase = RobotMap.armBase;
		topArm = RobotMap.topArm;
	}
	public void top(double speed){
		topArm.set(speed);
	}
	public void base(double speed){
		armBase.set(speed);	
	}

}
