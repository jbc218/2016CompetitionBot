package org.usfirst.frc.team4910.robot;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.nio.ByteBuffer;

import org.usfirst.frc.team4910.robot.Robot.RobotStatus;
import org.usfirst.frc.team4910.robot.commands.*;
import org.usfirst.frc.team4910.robot.subsystems.*;
//import org.usfirst.frc.team4910.robot.subsystems.VisionCV;
import org.usfirst.frc.team4910.util.Units;

//import com.ni.vision.NIVision;
//import com.ni.vision.NIVision.Image;

import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.Timer;
import com.ctre.CANTalon.*;
import edu.wpi.first.wpilibj.Timer.Interface;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import javafx.geometry.Pos;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 * 
 * In short, this means that the methods in this class are called by the main
 * library, in RobotBase. They do not call them in separate threads, so if you want
 * to multithread (i.e. handle two loops at the same time), you have to actually 
 * handle that yourself. It's painful to do so.
 * 
 * Finally, before reading, ignore any deprecated classes or code (code with srikethrough)
 */
@SuppressWarnings("unused")
public class Robot extends IterativeRobot {
	//ianchar13oneau@gmail.com
	
	/**Commands and subsystems created here*/
	public static OI oi;
	public static Shooter s;
	public static Intake in;
	public static DriveTrain driveTrain;
	public static DriveAngle driveAngle;
	public static DriveDistance driveDistance;
	public static DriveSpeed driveSpeed;
//	SendableChooser send;
    Command collectCommand;
    Command shootCommand;
    CameraServer cam; //CameraServer can mean axis camera or USB camera, but it's the actual camera object
    Command autonomousCommand;
    public static SendableChooser autoChoose; //SendableChooser is something that displays to the
    //SmartDashboard to allow the drivers to click a radio button (selection button) to change a setting.
    //An object of the SendableChooser essentailly means another radio button selection
    
    //The following objects are no longer needed and can be removed.
    public static boolean balanced;
    public static boolean collectEnd=true;
    private static double N = RobotMap.magicN;
    private static double E = RobotMap.magicE;
    private static double M = RobotMap.magicM;
    public static boolean hasIterated=false;
    public static boolean armCommandActive=false;
    private static double disabledTime=0;

    /**For testing only. We had so many trials that displaying them in the terminal would be pointless*/
//    File file, file2;
//    BufferedWriter bw,bw2;
//    FileWriter fw,fw2;

    
    /**Multiple cameras, only used if vision tracking isn't*/ //very useful code for following years
//  public static SendableChooser cameraSelector; //allows you to select which camera to use
//	static String lastSelected = ""; //forgot why I had this, but it can be deleted
//	static int curCam; //Should make sense, but the "int" is just the camera number. Each camera is either cam0, cam1, etc. They all call dev/video0, dev/video1, etc.
//	static int sessionMain;
//	static int sessionAux;
//	Image frame; //NIVision image, not openCV. You cannot treat them the same, incase that wasn't obvious
	
    /**Robot status
     * 
     * These will only update if you are not currently inside a loop. For example,
     * if you were to, in teleopPeriodic(), put code that says "while(currentStatus.equals(RobotStatus.TELEOP)){}" and then
     * hit the disable button or the enter key, that loop will not break. It will only break if something is running in a separate thread from
     * the main thread. I haven't implemented anything that would use this yet.
     */
	public enum RobotStatus{
		TELEOP,DISABLED,AUTONOMOUS,INIT
	}
	public static RobotStatus currentStatus=RobotStatus.INIT;
    
    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     * 
     * Any errors that pop up here will not actually show, causing the robot to be stuck, and causing the unsuspecting
     * programmer to be confused as to why nothing is popping up in the console. This may or may not be a bug for
     * WPILib, but just know to look through here if you're crashing without an error.
     */
    public void robotInit() {
    	currentStatus=RobotStatus.INIT;
    	System.out.println("RobotInit");
    	
    	/**init all subsystems and instances*/ 
    	//As a side note, the fact that you have to create objects for things that aren't really there is a major
    	//criticism of OOP.
		RobotMap.init();
		s = Shooter.getInstance(); //deprecated, replaced with a command (see OI for more)
		in = Intake.getInstance(); //deprecated, replaced with a command
		driveAngle = new DriveAngle(); //tested, but I don't trust it because of the many other failures of other objects with similar structure
		driveDistance = new DriveDistance(); //not thoroughly tested
		driveSpeed = new DriveSpeed(); //not thoroughly tested
		driveTrain = DriveTrain.getInstance();
		oi = new OI();
		//VisionCV.init(); //shouldn't be uncommented unless you're testing it
		/**Auto chooser*/
		autoChoose = new SendableChooser(); //explained above, allows you to choose which auto mode you want
		autoChoose.addObject("POSITION CHOOSER", "0");
		autoChoose.addDefault("Everything that isn't low bar", "1");
		autoChoose.addObject("Touch", "2");
		autoChoose.addObject("Low bar with low goal", "3");
		autoChoose.addObject("Low Bar, High Goal", "5"); //doesn't actually work, but I kept forgetting to remove it
		autoChoose.addObject("Do Nothing", "9");
		SmartDashboard.putData("Auto mode", autoChoose);

		/**Camera selector, only used if vision tracking isn't*/
//		cameraSelector = new SendableChooser();
//		cameraSelector.addObject("Camera Selector", "Camera Selector");
//		cameraSelector.addDefault("Main View", "Main View");
//		cameraSelector.addObject("Aux View", "Aux View");
//		SmartDashboard.putData("Camera Selector", cameraSelector);
		
		
		
		
		/**Use this if one camera is completely allocated to vision tracking, or if you only have one viewable camera (i.e. no camSelector)*/
		cam = CameraServer.getInstance(); 
	    //cam.setQuality(10); //they don't mind low quality video
	    //cam.startAutomaticCapture("cam1");
		//only commented out because it doesn't work with the new libraries, this was uncommented as of the last time we ran this, whenver that was
		
	    currentStatus=RobotStatus.DISABLED;
    }

	public void disabledPeriodic() {
		Scheduler.getInstance().run(); //no, I have no idea what this line does. It's here when you create the project, though.
		//nothing bad happens when you delete that line (it's in every method in this class), but I imagine it's used for something.
		
	}
	public int target = 1;
	public void autonomousInit() {
		//Please do not repeat any of this type of code you see below.
		//Essentially, the entire autonomous program is layed out here (which could've been in another class)
		//Some of this is deprecated, and I wouldn't repeat a lot of this next year. Most of this should be obvious, and I'll comment
		//wherever it needs it, but realize this went through a ton of development and testing, so the code won't flow as great as it could
		
	    currentStatus=RobotStatus.AUTONOMOUS;
		int pos;
		if(!(((String)autoChoose.getSelected()).equals("POSITION CHOOSER") || ((String)autoChoose.getSelected()).equals("Do Nothing"))){
			pos = Integer.parseInt((String) autoChoose.getSelected());
		}else{pos=0;}
		Scheduler.getInstance().run();
		RobotMap.LD1.changeControlMode(TalonControlMode.Position); //Changes from percentVBUS to position, where percentVBUS is the percentage of input voltage
		RobotMap.RD1.changeControlMode(TalonControlMode.Position);
    	RobotMap.RD1.setPID(RobotMap.PIDRP.KpFast, RobotMap.PIDRP.KiFast, RobotMap.PIDRP.KdFast); //yes, CANTalon has PID implemented into it.
    	RobotMap.LD1.setPID(RobotMap.PIDLP.KpFast, RobotMap.PIDLP.KiFast, RobotMap.PIDLP.KdFast); //Looking back, I forgot to set these to 0 at the end. This could've caused problems but it didn't seem to.
		RobotMap.LD1.setEncPosition(0);
		RobotMap.RD1.setEncPosition(0); //resets encoder, I'll explain what that is later, but it counts parts of a revolution
		RobotMap.g.reset(); //resets gyro
		//Units.set(RobotMap.LD1, 125.0);
		//Units.set(RobotMap.RD1, -125.0);
		//DriveTrain.driveAngleAndDistance(0, -92.0);
		System.out.println("Target: " +target+ ", Pos: " +pos);
		//LD1
		if(target == 1) { //Intake ball other defences (old comment, I think I wrote this late at night a while ago since this comment makes no sense)
			//Forward 125 inches (old notes)
			//DriveTrain.driveAngleAndDistance(0, -144.0); (old comment)
	        double kP1=RobotMap.PIDAT.KpFast;
	        double kI1=RobotMap.PIDAT.KiFast;
	        double kD1=RobotMap.PIDAT.KdFast;
	        double kP2=RobotMap.PIDAB.KpFast;
	        double kI2=RobotMap.PIDAB.KiFast;
	        double kD2=RobotMap.PIDAB.KdFast;
	        //these following lines aren't too bad, and it's actually what you should do
	    	RobotMap.ATPID = new PIDController(kP1, kI1, kD1, RobotMap.topEnc, RobotMap.armBase);
			RobotMap.topEnc.reset();
			RobotMap.ATPID.setSetpoint(-20);
			RobotMap.ATPID.enable();
			Timer.delay(.1);
			RobotMap.ATPID.disable(); //while I could just keep it enabled and change the setpoint at different times, I wanted to make
			//100% sure that this wouldn't spaz out after another motor changes. I don't know why I didn't just use the onTarget method
			//instead of the timer, but I will try that out some later time.
			RobotMap.topEnc.reset();
			if(pos == 1) { 
				/******************************/
				//This probably isn't the best way to do it, but we basically just went through trial and error to get all the following numbers.
				//We could've tried measuring the "resistance" of the ball on the wheels and calculated distance from there, but don't tell
				//that to the build mentors or Matthew. We should do that this year.
	        	RobotMap.INA.set(-.5); 
	        	RobotMap.shoot.set(-.9);
	        	Timer.delay(.19); //this and above starts the shooter wheel and shoots the shield down
	        	RobotMap.INA.set(0); //stops intake angle or shield
	    		RobotMap.ATPID.setSetpoint(35); //moves the arm back towards the shield a little
	    		RobotMap.ATPID.enable();
	    		Timer.delay(.1); //you'd be surprised as to how quickly .1 second is, it was a miricle watching it for the first time
	    		//as the mentors were speechless after I completely guessed the numbers 35, .1, -186, and .8 all at once
	    		RobotMap.ATPID.disable();
	        	RobotMap.ATPID.setSetpoint(-186); //moves it back down and slams it onto the motor
	        	RobotMap.ATPID.enable();
	        	Timer.delay(.8);
	        	RobotMap.ATPID.disable();
	        	RobotMap.shoot.set(-1); //go faster because of the resistance the ball puts, this is probably useless but it didn't hurt
	        	RobotMap.INS.set(-.1); //moves the intake wheels
	        	Timer.delay(.18);
	        	RobotMap.shoot.set(0);
	        	RobotMap.INS.set(0);
				RobotMap.LD1.changeControlMode(TalonControlMode.PercentVbus);
				RobotMap.RD1.changeControlMode(TalonControlMode.PercentVbus);
				//the ball is now securely in the robot with the shield and arm down
				DriveTrain.drive(.6, -.6); //move at half speed forward
				Timer.delay(3.0); //for 3 seconds (rather than a set distance which we should've done, but never got working)
	        	DriveTrain.drive(0, 0); //stops
				
			}else if(pos == 2) {
				//same thing down to next comment
	        	RobotMap.INA.set(-.5);
	        	RobotMap.shoot.set(-.9);
	        	Timer.delay(.19);
	        	RobotMap.INA.set(0);
	    		RobotMap.ATPID.setSetpoint(35);
	    		RobotMap.ATPID.enable();
	    		Timer.delay(.1);
	    		RobotMap.ATPID.disable();
	        	RobotMap.ATPID.setSetpoint(-186);
	        	RobotMap.ATPID.enable();
	        	Timer.delay(.8);
	        	RobotMap.ATPID.disable();
	        	RobotMap.shoot.set(-1);
	        	RobotMap.INS.set(-.1);
	        	Timer.delay(.18);
	        	RobotMap.shoot.set(0);
	        	RobotMap.INS.set(0);
	        	RobotMap.LD1.changeControlMode(TalonControlMode.PercentVbus);
				RobotMap.RD1.changeControlMode(TalonControlMode.PercentVbus);
				
				DriveTrain.drive(.3, -.3); //this was meant to slowly go forward until it touched the defence
				Timer.delay(1.0); //and it failed
				DriveTrain.drive(0, 0);
	        	RobotMap.LD1.setEncPosition(0); //copied and pasted from other auto mode, so that's why this is here (it shouldn't be)
	        	RobotMap.RD1.setEncPosition(0);

			}else if(pos == 3) { //Low Goal Low Bar
	        	RobotMap.INA.set(-.5);
	        	RobotMap.shoot.set(-.9);
	        	Timer.delay(.19);
	        	RobotMap.INA.set(0);
	    		RobotMap.ATPID.setSetpoint(35);
	    		RobotMap.ATPID.enable();
	    		Timer.delay(.1);
	    		RobotMap.ATPID.disable();
	        	RobotMap.ATPID.setSetpoint(-186);
	        	RobotMap.ATPID.enable();
	        	Timer.delay(.8);
	        	RobotMap.ATPID.disable();
	        	RobotMap.shoot.set(-1);
	        	RobotMap.INS.set(-.1);
	        	Timer.delay(.18);
	        	RobotMap.shoot.set(0);
	        	RobotMap.INS.set(0);
				RobotMap.LD1.changeControlMode(TalonControlMode.PercentVbus);
				RobotMap.RD1.changeControlMode(TalonControlMode.PercentVbus);
				
				DriveTrain.drive(-.5, .5); //half speed backwards under the low bar
				Timer.delay(3.0); //until it's around the tower-ish
	        	DriveTrain.drive(0, 0);
	        	DriveTrain.driveAngleAndDistance(49, 0); //turn to 45 degrees, except I had to make it 49 to make it work.
	        	//Please don't use that method or any code from that method. It's all just a mess.
	    		DriveTrain.drive(0, 0);
	    		
			}else if(pos == 5) { //Low bar, high goal
				
			}else if(pos == 9) { //Intake ball and do nothing
	        	RobotMap.INA.set(-.5);
	        	RobotMap.shoot.set(-.9);
	        	Timer.delay(.19);
	        	RobotMap.INA.set(0);
	    		RobotMap.ATPID.setSetpoint(35);
	    		RobotMap.ATPID.enable();
	    		Timer.delay(.1);
	    		RobotMap.ATPID.disable();
	        	RobotMap.ATPID.setSetpoint(-186);
	        	RobotMap.ATPID.enable();
	        	Timer.delay(.8);
	        	RobotMap.ATPID.disable();
	        	RobotMap.shoot.set(-1);
	        	RobotMap.INS.set(-.1);
	        	Timer.delay(.18);
	        	RobotMap.shoot.set(0);
	        	RobotMap.INS.set(0);
				RobotMap.LD1.changeControlMode(TalonControlMode.PercentVbus);
				RobotMap.RD1.changeControlMode(TalonControlMode.PercentVbus);

			}else if(pos == 5) {
				
			}
		}else if(target == 2) {
			
			if(pos == 1){
				
			}else if(pos == 2) {
				
			}
		}


		/**This is the only copy I will keep.*/
		//it logged data for me (or matlab) to analyze. Look through it if you like, but it's nothing worth talking about.
		/*
		
		try{
			file = new File("/U/PIDL.txt");
			file2 = new File("/U/PIDR.txt");
			fw = new FileWriter(file);
			fw2 = new FileWriter(file2);
		}catch(IOException e){
			e.printStackTrace();
		}
		bw = new BufferedWriter(fw);
		bw2 = new BufferedWriter(fw2);
		for(int i=0;i<=10;i++){
			//long a=System.currentTimeMillis();
			try{
				RobotMap.LD1.changeControlMode(TalonControlMode.Position);
				RobotMap.RD1.changeControlMode(TalonControlMode.Position);
				//Speed
		    	RobotMap.RD1.setPID(RobotMap.PIDRP[0], RobotMap.PIDRP[1], RobotMap.PIDRP[2]);
		    	RobotMap.LD1.setPID(RobotMap.PIDLP[0], RobotMap.PIDLP[1], RobotMap.PIDLP[2]);
				RobotMap.LD1.set((i*512));
				RobotMap.RD1.set((i*512));
				Timer.delay(2);
				bw.write((i*512)+" "+Math.abs(RobotMap.LD1.getPosition())+"\n");
				bw2.write((i*512)+" "+Math.abs(RobotMap.RD1.getPosition())+"\n");
				System.out.println((i*512)+" "+Math.abs(RobotMap.LD1.getPosition()));
				System.out.println((i*512)+" "+Math.abs(RobotMap.RD1.getPosition()));
				RobotMap.LD1.setEncPosition(0);
				RobotMap.RD1.setEncPosition(0);
				Timer.delay(.28);
				bw.newLine();
				bw2.newLine();
				//System.out.println(System.currentTimeMillis()-a);
			
			
			}catch(IOException e){
				e.printStackTrace();
			}	
		}
		try {
			bw.close();
			bw2.close();
			fw.close();
			fw2.close();
			System.out.println("Done");
		} catch (IOException e) {
			e.printStackTrace();
		}*/
		/**End of that code*/
		
		if (autonomousCommand != null)
			autonomousCommand.start();
		
	}

    /**
     * This function is called periodically during autonomous
     */
    public void autonomousPeriodic() { //does not actually do anything because everything's in init()
        Scheduler.getInstance().run();
    }

    public void teleopInit() { //constructs whatever disabledInit() destructed as it pertains to teleop.
	    currentStatus=RobotStatus.TELEOP;
        if (autonomousCommand != null) autonomousCommand.cancel();
        if(collectCommand==null) collectCommand = new Collect(); //ScreenStepsLive put it here, so I'm putting this here. It just creates the object.
        if(shootCommand==null) shootCommand = new Shoot();
        RobotMap.INA.setEncPosition(0); //all encoder positions should be set to 0 here.
        armCommandActive=false;
		DriveTrain.destructor(); //pretend this line isn't here, I haven't tested the stuff that goes with it yet.
    	RobotMap.g.reset(); //Everything should be set to 0 for now
		RobotMap.topEnc.reset(); //this is how you reset an encoder
		RobotMap.LD1.setEncPosition(0);
		RobotMap.RD1.setEncPosition(0);
		RobotMap.LD1.changeControlMode(TalonControlMode.PercentVbus);
    	RobotMap.RD1.changeControlMode(TalonControlMode.PercentVbus);
//		TalonControlMode Tl = RobotMap.LD1.getControlMode();
//    	TalonControlMode Tr = RobotMap.RD1.getControlMode();
//    	RobotMap.LD1.changeControlMode(TalonControlMode.Speed);
//    	RobotMap.RD1.changeControlMode(TalonControlMode.Speed);
//    	RobotMap.LD1.setEncPosition(0);
//    	RobotMap.RD1.setEncPosition(0);
    }
    /**
     * This function is called when the disabled button is hit.
     * You can use it to reset subsystems before shutting down.
     */
    public void disabledInit(){
	    currentStatus=RobotStatus.DISABLED;
    	RobotMap.INA.set(0);
    	
    	//to explain the following lines, just understand that the hasIterated bool wasn't set up correctly
    	//RobotMap.topEnc.reset();
//    	if(hasIterated){ //this routinely crashes and I have no idea why but we're not using the arm anyway, plus the PID code is eventually going to change
//    		RobotMap.ABPID.disable();
//    		RobotMap.ATPID.disable();
//    		hasIterated=false;
//    	}
    	if(!DriveTrain.currentDriveType.equals(DriveTrain.driveType.PVBUS)){} //I forgot what I was going to do here, I think I was going to destruct it, (but isn't needed)
    	armCommandActive=false; //not needed
    	disabledTime=Timer.getFPGATimestamp(); //just in case you care about total time enabled (no idea why anyone cares about that sort of thing)
    }
    /**
     * This function is called periodically during operator control
     */
    public void teleopPeriodic() {
    	/**I don't know if it's safe to remove this useless line, but it doesn't seem to harm the robot*/ //old comment
        Scheduler.getInstance().run();
        
    	
    	/**Handle vision for testing or possibly using at GRITS*/ //looks like I was wrong about that
    	//VisionCV.processImage(); //Mainly for testing, but would've been used to automate the shootng process
    	//System.out.println(VisionCV.insideTarget);
    	//if(VisionCV.insideTarget && OI.leftStick.getTrigger() && OI.rightStick.getTrigger()) s.ShootCommand(); //not actually sure if that would've worked, haven't tested it. I hope it would.

        
        /**No use wasting a full command on this*/ //you'll understand after reading OI why I wouldn't make an entire class to do this
//        if(OI.armStick.getButton(1)){
//        	RobotMap.g.reset();
//        }
        //if(OI.armStick.getButton(10))RobotMap.g.calibrate(); //Only uncomment that if you need to do a bunch of testing
        
        /**Driving*/
        DriveTrain.drive(-OI.leftStick.getY(), OI.rightStick.getY()); //yes, one side is always negative because one motor is backwards
        
        
        /**Arm stuff, deprecated*/
//        if(OI.armStick.getThrottle()<0.3){ //if the lever thing on the joystick is greater than 30% of the way up
//        		//System.out.println("basepot: "+RobotMap.basePot.get());
//            	//System.out.println("topenc: "+RobotMap.topEnc.get()+" topdeg: "+Units.CTDeg(RobotMap.topEnc.get()));
//            	
//           	RobotMap.armBase.set(OI.armStick.getY()); //yes, it would actually set the motors to a percent vbus reading
//           	RobotMap.topArm.set(OI.shootStick.getY()); //you can see why this is dangerous
//        		if((OI.armStick.getY()>0.0 && RobotMap.basePot.get()>149) || (OI.armStick.getY()<0.0 && RobotMap.basePot.get()<-22)){
//        			RobotMap.armBase.set(0); //that condition sort of worked. The motor still got damaged and blew up anyway because of our failed testing
//        		}else{
//        			RobotMap.armBase.set(OI.armStick.getY());
//        		}
//        		if((OI.shootStick.getY()>0.0 && RobotMap.topEnc.get()>468) || (OI.shootStick.getY()<0.0 && RobotMap.topEnc.get()<-331)){
//        			RobotMap.topArm.set(0);
//        		}else{
//        			RobotMap.topArm.set(OI.shootStick.getY());
//        		}
//        		
//        }else{
//           	//RobotMap.topArm.set(0);
//           	//RobotMap.armBase.set(0);
//        }
        /**End arm stuff*/

        /**Camera selector code*/
//        /* //this was actually commented out, it allowed the selector to be used to switch rather than just a joystick control
//        try{
//    		if(cameraSelector.getSelected().equals("Main View")){
//    			NIVision.IMAQdxConfigureGrab(sessionMain);
//    			NIVision.IMAQdxGrab(sessionMain, frame, 1);
//    			CameraServer.getInstance().setImage(frame);
//    		}else if(cameraSelector.getSelected().equals("Aux View")){
//    			NIVision.IMAQdxConfigureGrab(sessionAux);
//    			NIVision.IMAQdxGrab(sessionAux, frame, 1);
//    			CameraServer.getInstance().setImage(frame);
//    		}
//        }catch(Exception e){
//        	e.printStackTrace();
//        	//an exception won't happen, but if it did, I wouldn't bother stopping the entire robot because a simple camera is broken
//        }
//        */
//		  //end of old comment
//
//        int temp=curCam; //this now allows the driver to control which camera is used with the joystick
//        boolean hasRan=false;
//        while(OI.leftStick.getButton(2)){ //switches the current cam
//        	hasRan=true;
//        	if(temp==sessionMain){
//        		curCam=sessionAux;
//        	}else{
//        		curCam=sessionMain;
//        	}
//        }  
//        try{
//			NIVision.IMAQdxConfigureGrab(curCam);
//			NIVision.IMAQdxGrab(curCam, frame, 1);
//			cam=CameraServer.getInstance();
//			cam.setQuality(60);
//			cam.setImage(frame);
//        }catch(Exception e){
//        	e.printStackTrace();
//        }
//    	  hasRan=false;
        /**End of the code the drivers asked repeatedly for but decided to remove the next day*/
        
        /**Begin (more) testing code*/ //Most of the following code wouldn't even work now because half of it is removed. You can ignore it, but I might use it eventually
        //essentially, it let you establish PID values by incrimenting P, I and D with joystick buttons. I'll likely reuse it in the future, and when that time comes, I'll tell you what I'm doing
        
//        int goalheading=90;
//        if(OI.armStick.getButton(12)){ //remember that the driving section is commented out
//        	while(OI.armStick.getButton(12));
//        	System.out.println("test");
//        	double begintime = Timer.getFPGATimestamp();
//        	DriveControllerTurnInPlace.setGoalHeading(goalheading);
//        	RobotMap.g.reset();
//        	while(!DriveControllerTurnInPlace.isFinished()){
//        		double u = DriveControllerTurnInPlace.calculate(true);
//        		DriveTrain.driveNT(u, u);
//        		System.out.print("E: "+(((int)(100.0*(DriveControllerTurnInPlace.getGoalHeading()-RobotMap.g.getAngle())))/100.0));
//        		System.out.print("\tU: "+Math.abs(((int)(10000.0*u))/10000.0));
//        		System.out.println();
//        		//System.out.println(Math.abs(RobotMap.LD1.get())+Math.abs(RobotMap.RD1.get()));
//        	}
//        	System.out.println(goalheading-RobotMap.g.getAngle());
//        	DriveControllerTurnInPlace.destructor();
//        	DriveTrain.driveNT(0, 0);
//        	//System.out.println(Timer.getFPGATimestamp()-begintime);
//        }
//        System.out.println(goalheading-RobotMap.g.getAngle());
        
        //System.out.println("Encoder L: "+RobotMap.LD1.getEncPosition()+"\nEncoder R: "+RobotMap.RD1.getEncPosition());

        
//        if(OI.armStick.getButton(12)){
//        	while(OI.armStick.getButton(12));
//        	RobotMap.LD1.changeControlMode(TalonControlMode.Position);
//        	RobotMap.RD1.changeControlMode(TalonControlMode.Position);
//        	RobotMap.LD1.set(Units.DTC(12));
//        	RobotMap.RD1.set(-Units.DTC(12));
//        }
//        if(OI.armStick.getButton(9)){
//        	while(OI.armStick.getButton(9));
//        	RobotMap.LD1.changeControlMode(TalonControlMode.Position);
//        	RobotMap.RD1.changeControlMode(TalonControlMode.Position);
//        	RobotMap.RD1.reverseSensor(true);
//        	b=true;
//        }
//        if(b){
//        	RobotMap.LD1.set(Units.DTC(6.0*OI.leftStick.getY()));
//        	System.out.println(Units.DTC(6.0*OI.leftStick.getY()));
//    		RobotMap.RD1.set(-Units.DTC(6.0*OI.leftStick.getY()));
//    		System.out.println(-Units.DTC(6.0*OI.leftStick.getY()));
//        }
//        if(OI.armStick.getButton(11)){
//        	while(OI.armStick.getButton(11));
//        	while(!OI.armStick.getButton(12) && !VisionCV.insideTarget){
//        		VisionCV.processImage();
//        	}
//        	VisionCV.insideTarget=false;
//        	System.out.println("Now inside target, press trigger again to redo");
//        }
        
//        if(OI.armStick.getButton(9) && DriveTrain.isFinished){
//        	while(OI.armStick.getButton(9));
//        	DriveTrain.driveToSmallAngle(-3);
//        }
//        if(DriveTrain.isTurning && driveAngle.onTarget()){
//        	driveAngle.getPIDController().reset();
//        	Timer.delay(.2);
//        	DriveTrain.isFinished=true;
//        }
//        if(DriveTrain.isTurning && !DriveTrain.isFinished){
//        	System.out.println(-3-driveAngle.getSetpoint());
//        }
        
        //DriveTrain.driveToSpeedCAN(OI.leftStick.getY()*10000.0);
        //DriveTrain.driveToSpeedCAN();
//        Timer.delay(0.2);
        //Drive2Train.drive(.146, -.145);
        /**Establishes minimum speeds*/ //still part of testing code, by the way.
//        if(OI.leftStick.getButton(9)){
//        	while(OI.leftStick.getButton(9));
//        	double start=.15;
//        	double incriment=.001;
//        	while(!OI.leftStick.getButton(10)){
//        		System.out.println(start);
//        		if(OI.leftStick.getButton(1)) 
//        			DriveTrain.driveNT(-start, -start); 
//        		else
//        			DriveTrain.driveNT(start, start);
//        		if(OI.leftStick.getButton(11)){
//        			while(OI.leftStick.getButton(11));
//        			incriment*=10.0;
//        		}
//        		if(OI.leftStick.getButton(12)){
//        			while(OI.leftStick.getButton(12));
//        			incriment/=10.0;
//        		}
//        		if(OI.leftStick.getButton(7)){
//        			while(OI.leftStick.getButton(7));
//        			start+=incriment;
//        		}
//        		if(OI.leftStick.getButton(8)){
//        			while(OI.leftStick.getButton(8));
//        			start-=incriment;
//        		}
//        	}
//        }
        /**End testing code*/
    }
    /**
     *	The following code was meant to update the camera and draw an image onto it using NIVision. It failed, but you could read through it if you want.
     */
   
//    public void updateCamera(){
//    	String selectedCam = (String) cameraSelector.getSelected();
//    	int id = NIVision.IMAQdxGrab(curCam,frame,1);
//		/*
//		 * 	STEPS:
//		 *  write files down onto roboRIO
//		 *  use opencv to read images, do processing, etc
//		 *  get bounding rectangle coordinates through opencv
//		 *  draw them with NI onto image
//		 *  display image
//		 * 
//		 */
//		for(int i=0;i<20;i++){
//			NIVision.imaqWriteJPEGFile(frame, "input"+Integer.toString(i), 50, null); //It's either this or new RawData() or I'm doing something wrong
//		}
//		org.opencv.core.Rect r = VisionCV.onlyGetRect(20);
//		NIVision.imaqAddRectContour(NIVision.imaqMaskToROI(frame).val, (new Rect(r.x,r.y,r.height,r.width)));
//    	CameraServer.getInstance().setImage(frame);
//    	//POSSIBLE ALTERNATIVE: run it coinciding rather than 20 at a time, however that could be slower
//    }
//    
    public void testInit() { //I have no idea what testInit is, but I do know that some idiot who worked on WPILib forces you to do any testing with LiveWindow (SmartDashboard except with network tables) to do it all during "test" period.
        //I have no idea how test mode is any different than teleop, but it's here if you ever figure it out.
    }
    public void testPeriodic() {
        LiveWindow.run(); //default code, again no idea why it's here.
    }
}
