package org.usfirst.frc.team4910.util;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;

public class GamepadF310 extends Joystick {  //NOTE: this is for the F310 gamepad
	private static final int leftX = 1;
	private static final int leftY = 2;
	private static final int trigger = 3; //one is 1, two is -1, both and neither are both 0
	private static final int rightX = 4;
	private static final int rightY = 5;
	private static final int dpadX = 6; //there is no dpad y axis
	private static final int A = 1;
	private static final int B = 2;
	private static final int X = 3;
	private static final int Y = 4;
	private static final int lBump = 5;
	private static final int rBump = 6;
	private static final int back = 7;
	private static final int start = 8;
	private static final int leftStickButton = 9;
	private static final int rightStickButton = 10;
	public GamepadF310(int port){
		super(port);
	}
	public double getLeftX(){
		return getRawAxis(leftX);
	}
	public double getRightX(){
		return getRawAxis(rightX);
	}
	public double getLeftY(){
		return getRawAxis(leftY);
	}
	public double getRightY(){
		return getRawAxis(rightY);
	}
	public double getTriggers(){
		return getRawAxis(trigger);
	}
	public double getDpadX(){
		return getRawAxis(dpadX);
	}
	/*
	 * Boolean valued buttons are here with lower-case method names
	 * Actual joy stick buttons are here with capital method names
	 */
	public boolean buttonA(){
		return getRawButton(A);
	}
	public boolean buttonB(){
		return getRawButton(B);
	}
	public boolean buttonX(){
		return getRawButton(X);
	}
	public boolean buttonY(){
		return getRawButton(Y);
	}
	public boolean buttonLBump(){
		return getRawButton(lBump);
	}
	public boolean buttonRBump(){
		return getRawButton(rBump);
	}
	public boolean buttonBack(){
		return getRawButton(back);
	}
	public boolean buttonStart(){
		return getRawButton(start);
	}
	public boolean buttonLStick(){
		return getRawButton(leftStickButton);
	}
	public boolean buttonRStick(){
		return getRawButton(rightStickButton);
	}
	public boolean DPadLeft(){
		return getDpadX()<-.5;
	}
	public boolean DPadRight(){
		return getDpadX()>.5;
	}
	public boolean leftTrigger(){
		return getTriggers()<-.5;
	}
	public boolean rightTrigger(){
		return getTriggers()>.5;
	}
	public JoystickButton ButtonA() {
		return new JoystickButton(this, A);
	}
	public JoystickButton ButtonB() {
		return new JoystickButton(this, B);
	}
	public JoystickButton ButtonX() {
		return new JoystickButton(this, X);
	}
	public JoystickButton ButtonY() {
		return new JoystickButton(this, Y);
	}
	public JoystickButton ButtonStart(){
		return new JoystickButton(this, start);
	}
	public JoystickButton ButtonBack() {
		return new JoystickButton(this, back);
	}
	public JoystickButton ButtonLeftBumper() {
		return new JoystickButton(this, lBump);
	}
	public JoystickButton ButtonRightBumper() {
		return new JoystickButton(this, rBump);
	}
	public JoystickButton ButtonLeftStick() {
		return new JoystickButton(this, leftStickButton);
	}
	public JoystickButton ButtonRightStick() {
		return new JoystickButton(this, rightStickButton);
	}
}