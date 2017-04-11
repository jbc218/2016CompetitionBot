package org.usfirst.frc.team4910.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.DriverStation.*;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;

import org.usfirst.frc.team4910.robot.RobotMap;
import org.usfirst.frc.team4910.robot.commands.*;
import org.usfirst.frc.team4910.robot.subsystems.*;
import org.usfirst.frc.team4910.util.*;

public class OI {
    public static OI instance = null;
    
    
    public static LJoystick rightStick; //LJoystick is a wrapper class. The purpose is to make it more programmer friendly.
    public static LJoystick leftStick;
    public static LJoystick armStick1; //for testing
    public static LJoystick armStick2; //for testing
    public static LJoystick shootStick;
    public static LJoystick intakeSStick; //not used
    public static LJoystick intakeAStick; //not used
//    public static LJoystick armStick; //recently removed
    public static Button b1,b2,b3,b4,b5,b6,b7,b8,b9,b10,b11,b12,b13,b14,b15;
    //while I could've actually just done: (new JoystickButton(shootStick,2)).whenActive(new Collect());, I chose readability over functionality.
    public OI(){
   	
        rightStick = new LJoystick(2);
        leftStick = new LJoystick(1);
        shootStick = new LJoystick(3);
//      armStick = new LJoystick(4); //again, removed recently
        b1 = new JoystickButton(shootStick,1); //nothing
        b2 = new JoystickButton(shootStick,2); //intakes ball
        b3 = new JoystickButton(rightStick,3); //slams shield into mechanical stop
        b4 = new JoystickButton(shootStick,4); //low goal
        b5 = new JoystickButton(shootStick,5); //nothing
        b6 = new JoystickButton(shootStick,6); //shoots
        b7 = new JoystickButton(rightStick,6); //moves intake "up"(forward) a little
        b8 = new JoystickButton(shootStick,8); //nothing
        b9 = new JoystickButton(shootStick,9); //nothing
        b10 = new JoystickButton(rightStick,4); //slams shield down
        b11 = new JoystickButton(rightStick,7); //nothing
        b12 = new JoystickButton(rightStick,5); //not actually used anymore, but it would've just set shield to 0
        //b11.whenPressed(new ArmDrawbridge());
        b2.whenPressed(new Collect()); //arm 2
        b3.whileHeld(new FullPowerIntake()); //right 3 //misleading name, it means IntakeUp.
        b4.whenPressed(new LowGoal()); //arm 4
        b6.whenPressed(new Shoot()); //arm 6
        b7.whenPressed(new IntakeBalance()); //right 6
        b10.whenPressed(new IntakeDown()); //right 4
        b12.whenPressed(new StopPowerToIntake()); //right 5
        //b1.whenPressed(new Testing());
        //b2.whenPressed(new ArmTest());
        //intakeSStick = new LJoystick(3);
        //intakeAStick = new LJoystick(4);
        
    	
    }
    
    
    public static OI getInstance(){ //only one instance of this class allowed, that's what this if statement handles.
    	return instance == null ? instance = new OI() : instance;
    }
}

