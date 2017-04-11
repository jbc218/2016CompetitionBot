package org.usfirst.frc.team4910.robot.subsystems;

import java.io.IOException;
import java.util.ArrayList;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;
import org.opencv.videoio.VideoCapture;
import org.usfirst.frc.team4910.robot.OI;
import org.usfirst.frc.team4910.robot.RobotMap;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Timer;
import com.ctre.CANTalon.TalonControlMode;
import edu.wpi.first.wpilibj.command.Subsystem;
@Deprecated
public class VisionCVTesting extends Subsystem {
	public static double horizontalFOV = 61.0;
	public static double verticalFOV = 34.3;
	public static double degPerPixel = horizontalFOV/640.0; //assuming 640x480
	public static double centerImg = 319.5; //where the middle of the image is
	public static double tapeWidth=5.0/3.0; //1 ft 8 inch, 1+8/12
	public static double tapeHeight=7.0/6.0; //1 ft 2 inch, 1+2/12
	public static double tapeThickness = 1.0/6.0; //2 inch, this is the "height" of the tape before being cut
	public static double aspectRatio = 10.0/7.0; // width/height, also equals 1.4285
	public static double yawAngleToTargetApprox(double error){return (error)*degPerPixel;}
	//FOVx = 2 atan(.5*640/focalLength)
	//focalLength = 640/(2*tan(FOVx/2))
	public static double focalLength = 640.0/(2.0*Math.tan(61.0/2.0));
	public static double yawAngleToTarget(double error){return Math.atan((error)/focalLength);}
	public static final Scalar 
		//Color values
		BLUE = new Scalar(255, 0, 0),
		GREEN = new Scalar(0, 255, 0),
		RED = new Scalar(0, 0, 255),
		WHITE = new Scalar(255, 255, 255),
		BLACK = new Scalar(0, 0, 0),

		//HSV Threshold
		LOWER_BOUNDS = new Scalar(65,131,71),
		UPPER_BOUNDS = new Scalar(94,255,255);
	
	private static final int MIN_CONTOUR_AREA = 300;

	public static VideoCapture videoCapture;
	public static Mat BGR, HSV, blur, threshold, clusters, hierarchy;
	public static boolean insideTarget = false;
	public static SerialPort cam;
	private static int iteration=0;
	private static Scalar color = BLACK;
	private static int lastRun=0;
	private static boolean hasRun=false;
	public static void init(){
		//In case someone in the future is trying to install OpenCV onto the robot but can't seem to do it, here's what you do
		//Get winSCP, host name is roboRIO-4910-FRC.local, port 22, user admin, no password
		//Follow instructions on https://github.com/robotpy/roborio-opencv
		//System.load("/usr/local/lib/libopencv_java310.so");
		System.load("/usr/local/lib/libopencv_java310.so");
		BGR = new Mat();
		HSV = new Mat();
		blur = new Mat();
		threshold = new Mat();
		clusters = new Mat();
		hierarchy = new Mat();
		
//		NetworkTable table = NetworkTable.getTable("SmartDashboard");
		
		//This command can only be executed once, so if it's the wrong camera, or the robot hasn't fully loaded, or the camera is currently being used, it cannot be corrected
    	try {
			Runtime.getRuntime().exec("v4l2-ctl -d /dev/video0 -c exposure_auto=1 -c exposure_absolute=1");
		} catch (IOException e) {
			e.printStackTrace();
		}
    	Timer.delay(.5);
		videoCapture = new VideoCapture(0);
		videoCapture.open(0);
		Timer.delay(.5);
		//It does it twice because OpenCV can mess with camera settings
    	try {
			Runtime.getRuntime().exec("v4l2-ctl -d /dev/video0 -c exposure_auto=1 -c exposure_absolute=1");
		} catch (IOException e) {
			e.printStackTrace();
		}
		
		insideTarget = false;
	}

	@SuppressWarnings("unused")
	public static void processImage(){
		
		System.out.println("Process Image");
		videoCapture.read(BGR);
		
		ArrayList<MatOfPoint> contours = new ArrayList<MatOfPoint>();
		//Probably don't need this
		//contours.clear();
		Imgcodecs.imwrite("output.png", BGR);
		if (!BGR.empty() && !insideTarget) {
			System.out.println("The camera didn't mess up!");
			Imgproc.cvtColor(BGR, HSV, Imgproc.COLOR_BGR2HSV);
			
			Rect target = new Rect(new Point(187, 0), new Point(416, 87));

			

			// Blur and filter out smaller blobs to improve accuracy
			Imgproc.GaussianBlur(HSV, blur, new Size(45, 45), 3);
			Core.inRange(HSV, LOWER_BOUNDS, UPPER_BOUNDS, threshold);

			// Remove noise
			//Imgproc.dilate(threshold, threshold, new Mat());
			//Imgproc.dilate(threshold, threshold, new Mat());
			//Imgproc.erode(threshold, threshold, new Mat());
			//Imgproc.erode(threshold, threshold, new Mat());

			//RETR_LIST = no parent/child relationship, treat them all like regular contours
			//RETR_EXTERNAL = Only care about the parents, anything internal is gone. You must do hierarchy.get(0,number), since none are embeded
			Imgproc.findContours(threshold, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
			//hierarchy is the map of contours inside others. If we did a loop and set an array called outerMost[x]=hierarchy[0,x],
			//then outerMost[0] would tell you what the next contour index is that is external. [1] would tell you previous,
			//[2] and [3] would both be -1 since they have to do with children ([2] = first child, [3] = parent of current contour), if N/A for any
			//then it'd return -1 for that index
			//hierarchy.get(layer,contour index), where 0 is external
			
			double contA;
			MatOfPoint approxf1 = new MatOfPoint();
			MatOfPoint2f mMOP2f1 = new MatOfPoint2f(); // Converted contours
			MatOfPoint2f mMOP2f2 = new MatOfPoint2f(); // approxPolyDP stored

			if (hierarchy.size().height > 0 && hierarchy.size().width > 0 && !insideTarget) {
				System.out.println("Contours found");
				//idx means current contour index number, after this iteration, it'll equal the next index that is external
				//that means that this cycles through contours
				for (int idx = 0; idx >= 0 && !insideTarget; idx = (int) hierarchy.get(0, idx)[0]) {
					
					contA = Imgproc.contourArea(contours.get(idx));
					// System.out.println(contA);
					//if(contA==0 && hasRun) insideTarget=true; //This could seriously mess up
					if (contA > MIN_CONTOUR_AREA && !insideTarget) {
						System.out.println("Large contour(s) found");
						// Draw raw contours
						//Imgproc.drawContours(BGR, contours, idx, GREEN);

						// Convert contours to MatOfPoint2f to work with approxPollyDP
						contours.get(idx).convertTo(mMOP2f1, CvType.CV_32FC2);

						// Make contour "solid"
						Imgproc.approxPolyDP(mMOP2f1, mMOP2f2, 7, true);
						//System.out.println(mMOP2f2.total());
						// If the object has 4 corners, draw "Square"
						//if (mMOP2f2.total() == 8) {
							//Imgproc.putText(BGR, "Target", new Point(BGR.width() / 2, BGR.height() / 2), Core.FONT_HERSHEY_SIMPLEX, 1.0, new Scalar(0, 0, 0));
						//}

						// Convert mMOP2f2 to Mat of Point Contours (because thats different that normal MoP)
						mMOP2f2.convertTo(contours.get(idx), CvType.CV_32S);

						// Convert to normal MoP
						mMOP2f2.convertTo(approxf1, CvType.CV_32S);

						// Make a rectangle from that
						Rect rec = Imgproc.boundingRect(approxf1);
						System.out.println(rec.area());
						//Middle of rec
						Point middleObj = new Point((rec.tl().x+rec.br().x)/2, (rec.tl().y+rec.br().y)/2);
						Point middleTarget = new Point((target.tl().x+target.br().x)/2, (target.tl().y+target.br().y)/2);

						if (mMOP2f2.total() > 4) { //I have to refine this
							//System.out.println(mMOP2f2.total());
							System.out.println("Target moving time");
							//Check to see if rec is inside target and has a certain area
							double errorX=0;
							double errorY=0;
							if(rec.tl().x<target.tl().x) errorX=target.tl().x-rec.tl().x;
							if(rec.br().x>target.br().x) errorX=target.br().x-rec.br().x; //negative on purpose
							if(rec.tl().y<target.tl().y) errorY=target.tl().y-rec.tl().y;
							if(rec.br().y>target.br().y) errorY=target.br().y-rec.br().y; //again, negative on purpose
							if(rec.tl().x > target.tl().x && rec.br().x < target.br().x && rec.tl().y > target.tl().y && rec.br().y < target.br().y) {
							//if(rec.tl().x >= target.tl().x && rec.br().x <= target.br().x && rec.tl().y >= target.tl().y && rec.br().y <= target.br().y) {
								DriveTrain.drive(0, 0);
								//new Shoot();
								System.out.println("IN");
								
								color = GREEN;
								
								
								Imgproc.circle(BGR, middleObj, 2, BLACK, 3);						//Draw a point in the middle of the contour
								Imgproc.circle(BGR, new Point(rec.br().x, rec.br().y), 2, RED, 3);	//Draw a point on the bottom right corner of contour
								Imgproc.rectangle(BGR, rec.br(), rec.tl(), BLACK, 1);				//Draw rectangle encompassing contour
								Imgproc.drawContours(BGR, contours, idx, BLUE);	
								
								Imgproc.rectangle(BGR, target.br(), target.tl(), color, 2);	
								
								Imgproc.rectangle(BGR, target.br(), target.tl(), color, 2);				//Draw target rectangle
								iteration++;
								
								insideTarget=true;
								Imgcodecs.imwrite(("output".concat(Integer.toString(iteration).concat(".png"))), BGR);
							}else if(rec.br().x > target.br().x) { //Turn Right (rec.br().x > target.br().x)
								System.out.println("Turn Right");
								color = RED;
							}else if(rec.tl().x < target.tl().x) { //Turn Left (rec.tl().x < target.tl().x)
								System.out.println("Turn Left");
								color = RED;
							}else if (rec.tl().y < target.tl().y) { //Move Back, target too high (rec.tl().y < target.tl().y)
								color = RED;
								System.out.println("Move Back");
							}else if(rec.br().y > target.br().y) { //Move Forward, target too low (rec.br().y > target.br().y)								
								System.out.println("Move Forward");
								hasRun=true;
								color = RED;
							}

							Imgproc.circle(BGR, middleObj, 2, BLACK, 3);						//Draw a point in the middle of the contour
							Imgproc.circle(BGR, new Point(rec.br().x, rec.br().y), 2, RED, 3);	//Draw a point on the bottom right corner of contour
							Imgproc.rectangle(BGR, rec.br(), rec.tl(), BLACK, 1);				//Draw rectangle encompassing contour
							Imgproc.drawContours(BGR, contours, idx, BLUE);						// Draw "solid" contours
							
						}else{
							color = RED;
						}
					}
				}
			}
			Imgproc.rectangle(BGR, target.br(), target.tl(), color, 2);				//Draw target rectangle
			
			iteration++;
			Imgcodecs.imwrite(("output".concat(Integer.toString(iteration).concat(".png"))), BGR);
		}
		
		//videoCapture.release();
	}

	@Override
	protected void initDefaultCommand() {
				
	}
	private static double kP = RobotMap.DriveTurnPID.KpFast;
	private static double kI = RobotMap.DriveTurnPID.KiFast;
	private static double kD = RobotMap.DriveTurnPID.KdFast;
	
	private static double goalHeading;
	private static double lastError;
	private static double sumError;
	private static boolean reset=true;
	private static boolean complete=false;
	private static double lastTimestamp;

	public static double calculatePIDOutput(double theta){
        double dt = RobotMap.dt;
        if (!reset) {
            double now = Timer.getFPGATimestamp();
            dt = now - lastTimestamp;
            lastTimestamp = now;
        } else {
            lastTimestamp = Timer.getFPGATimestamp();
        }
		double error = theta-RobotMap.g.getAngle();
        if (reset) {
            // Prevent jump in derivative term when we have been reset.
            reset = false;
            lastError = error;
            sumError = 0;
        }
        double output = kP*error + kD*((error-lastError)/dt);
        if(output>-1.0 && output<1.0){
        	sumError+=error*dt;
        }
        output+=kI*sumError;
        lastError=error;
        if(Math.abs(RobotMap.g.getAngle() - theta) < 1 && Math.abs(RobotMap.LD1.get())+Math.abs(RobotMap.RD1.get()) < .01) reset=complete=true;
		return output;
	}	
}
