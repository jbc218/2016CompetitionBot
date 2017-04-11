package org.usfirst.frc.team4910.robot.commands;

import org.usfirst.frc.team4910.robot.OI;
import org.usfirst.frc.team4910.robot.Robot;
import org.usfirst.frc.team4910.robot.RobotMap;
import org.usfirst.frc.team4910.robot.subsystems.DriveTrain;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

public class IntakeDown extends Command{
		private static boolean isFinished;
	    private static double N = RobotMap.magicN;
	    private static double E = RobotMap.magicE;
	    private static double M = RobotMap.magicM;
		public IntakeDown(){
			requires(Robot.in);
		}
		@Override
		protected void initialize() {
			isFinished=false;
			while(OI.rightStick.getButton(4)){ RobotMap.INA.set(-.5); DriveTrain.drive(-OI.leftStick.getY(), OI.rightStick.getY());}
			RobotMap.INA.set(0);
			end();
			isFinished=true;
		}
		
		@Override
		protected void execute() {
			// TODO Auto-generated method stub
			
		}

		@Override
		protected boolean isFinished() {
			// TODO Auto-generated method stub
			return isFinished;
		}

		@Override
		protected void end() {
			// TODO Auto-generated method stub
			RobotMap.INA.set(0);
			isFinished=true;
		}

		@Override
		protected void interrupted() {
			// TODO Auto-generated method stub
			RobotMap.INA.set(0);
			isFinished=true;
		}
	}
