package org.usfirst.frc.team4910.robot.subsystems;

import org.usfirst.frc.team4910.robot.Robot;
import org.usfirst.frc.team4910.robot.RobotMap;

import com.ctre.CANTalon;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Subsystem;

public class Shooter extends Subsystem{
	private static Shooter instance;
	public static CANTalon shoot;
	@Override
	protected void initDefaultCommand() {
		// TODO Auto-generated method stub
		
	}
	
	public static Shooter getInstance(){
		return instance==null ? new Shooter() : instance;
	}
	
	public Shooter(){
		shoot = RobotMap.shoot;
	}
	public void Shoot(double speed){
		shoot.set(speed);
	}
	public void ShootCommand(){ //Only used in Teleop
		Robot.in.wheels(.425);
		RobotMap.INA.set(-.1);
		Timer.delay(.25);
		Robot.in.wheels(0);
		Robot.s.Shoot(1);
		Timer.delay(1.5);
		Robot.in.wheels(-1);
		Timer.delay(1);
		RobotMap.INA.set(0);
		Robot.s.Shoot(0);
		Robot.in.wheels(0);
	}
}
