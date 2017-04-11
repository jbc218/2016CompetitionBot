package org.usfirst.frc.team4910.util;

import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.Joystick;

public class GamepadDA extends Joystick { //dual action gamepad
	private static final int leftX = 1;
	private static final int leftY = 2;
	private static final int rightX = 3;
	private static final int rightY = 4;
	private static final int dpadX = 5;
	private static final int dpadY = 6;
	private static final int A = 1;
	private static final int B = 2;
	private static final int X = 3;
	private static final int Y = 4;
	private static final int lBump = 5;
	private static final int rBump = 6;
	private static final int leftTrigger = 7;
	private static final int rightTrigger = 8;
	private static final int back = 9;
	private static final int start = 10;
	private static final int leftStickButton = 11;
	private static final int rightStickButton = 12;
	
	public GamepadDA(int port){
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
	public double getDpadX(){
		return getRawAxis(dpadX);
	}
	public double getDpadY(){
		return getRawAxis(dpadY);
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
	public boolean DPadUp(){
		return getDpadY() < -.5;
	}
	public boolean DPadDown(){
		return getDpadY() > .5;
	}
	public boolean DPadLeft(){
		return getDpadX()<-.5;
	}
	public boolean DPadRight(){
		return getDpadX()>.5;
	}
	public boolean leftTrigger(){
		return getRawButton(leftTrigger);
	}
	public boolean rightTrigger(){
		return getRawButton(rightTrigger);
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