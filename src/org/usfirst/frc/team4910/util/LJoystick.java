package org.usfirst.frc.team4910.util;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;

public class LJoystick extends Joystick {
	public LJoystick(int port){
		super(port);
	}
	//Because LJoystick extends Joystick, getY() and getX() are inherited, I don't need methods for that.
	public boolean getButton(int b){
		return getRawButton(b);
	}
	public JoystickButton button(int b){
		return new JoystickButton(this,b);
	}
	public boolean povUp(){
		return getPOV(0)==0;
	}
	public boolean povDown(){
		return getPOV(0)==180;
	}
	public boolean povLeft(){
		return getPOV(0)==270;
	}
	public boolean povRight(){
		return getPOV(0)==90;
	}
}
