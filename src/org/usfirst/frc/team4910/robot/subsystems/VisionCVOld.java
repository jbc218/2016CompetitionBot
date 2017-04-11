package org.usfirst.frc.team4910.robot.subsystems;
//pretend I put a @Deprecated here
//package org.usfirst.frc.team4910.robot.subsystems;
//
//import java.io.File;
//import java.io.IOException;
//import java.sql.Blob;
//import java.util.ArrayList;
//import java.util.Iterator;
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//import org.opencv.core.Core;
//import org.opencv.core.Mat;
//import org.opencv.core.MatOfPoint;
//import org.opencv.core.Point;
//import org.opencv.core.Rect;
//import org.opencv.core.Scalar;
//import org.opencv.core.Size;
//import org.opencv.imgcodecs.Imgcodecs;
//import org.opencv.imgproc.Imgproc;
//import org.opencv.videoio.VideoCapture;
//import org.usfirst.frc.team4910.robot.OI;
//import org.usfirst.frc.team4910.robot.Robot;
//
//import com.ni.vision.NIVision;
//import com.ni.vision.NIVision.Image;
//
//import edu.wpi.first.wpilibj.SerialPort;
//import edu.wpi.first.wpilibj.SerialPort.Port;
//import edu.wpi.first.wpilibj.command.Subsystem;
//import edu.wpi.first.wpilibj.networktables.NetworkTable;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//
//public class VisionCV extends Subsystem {
//	public static final Scalar 
//	RED = new Scalar(0, 0, 255),
//	BLUE = new Scalar(255, 0, 0),
//	GREEN = new Scalar(0, 255, 0),
//	BLACK = new Scalar(0,0,0),
//	YELLOW = new Scalar(0, 255, 255),
////	these are the threshold values in order 
//	LOWER_BOUNDS = new Scalar(76,21,255),
//	UPPER_BOUNDS = new Scalar(88,72,255);
//	public static final Size resize = new Size(320,240);
//	public static VideoCapture videoCapture;
//	public static Mat matOriginal, matHSV, matThresh, clusters, matHeirarchy;
//	
////	the height to the top of the target in first stronghold is 97 inches	
//	public static final double TOP_TARGET_HEIGHT = 97.0; //checked
////	the physical height of the camera lens
//	public static final double TOP_CAMERA_HEIGHT = 10.3; //practice bot only
//	
////	camera details, check datasheet
//	public static final double VERTICAL_FOV  = 33.5828; //done
//	public static final double HORIZONTAL_FOV  = 59.7029; //done
//	public static final double CAMERA_ANGLE = 10.0; //practice bot only
//	
//	public static SerialPort cam;
//	static int iteration=0;
//	@SuppressWarnings("unused")
//	public static void init(){
//		System.load("/usr/local/lib/libopencv_java310.so");
//		matOriginal = new Mat();
//		matHSV = new Mat();
//		matThresh = new Mat();
//		clusters = new Mat();
//		matHeirarchy = new Mat();
//		NetworkTable table = NetworkTable.getTable("SmartDashboard");
//		//cam = new SerialPort(0,Port.kUSB);
//		
//		/*try{
//        	Runtime.getRuntime().exec(Robot.GRIP_ARGS);
//        	System.out.println("GRIP initiated");
//        }catch(IOException e){
//        	e.printStackTrace();
//        	System.out.println("Could not execute grip");
//        }*/
//		
//		//TODO: remember to change this for the NIVision stuff
//		videoCapture = new VideoCapture(0);
//		videoCapture.open(0);
//		
//	}
//	public static void auto(){
//		try{
//			//while(!videoCapture.isOpened());// System.out.println(videoCapture.isOpened());
//			if(OI.shootStick.getButton(9)) processImage();
//		}catch(Exception e){
//			e.printStackTrace();
//			System.out.println("Error in VisionCV");
//		}
//	}
//	static double average=0;
//	static int asdfasdf=0;
//	@SuppressWarnings("unused")
//	public static void processImage(){
//		System.out.println("Hello World!");
//		ArrayList<MatOfPoint> contours = new ArrayList<MatOfPoint>();
//		double x,y,targetX,targetY,distance,azimuth;
////		frame counter
//		int FrameCount = 0;
//		long before = System.currentTimeMillis();
////		only run for the specified time
//		while(FrameCount < 100){
//			contours.clear();
////			capture from the axis camera
//			videoCapture.read(matOriginal);
////			captures from a static file for testing
////			matOriginal = Imgcodecs.imread("someFile.png");
//			Imgproc.cvtColor(matOriginal,matHSV,Imgproc.COLOR_BGR2HSV);			
//			Core.inRange(matHSV, LOWER_BOUNDS, UPPER_BOUNDS, matThresh);
//			Imgproc.findContours(matThresh, contours, matHeirarchy, Imgproc.RETR_EXTERNAL, 
//					Imgproc.CHAIN_APPROX_SIMPLE);
////			make sure the contours that are detected are at least 20x20 
////			pixels with an area of 400 and an aspect ration greater then 1
//			for (Iterator<MatOfPoint> iterator = contours.iterator(); iterator.hasNext();) {
//				MatOfPoint matOfPoint = (MatOfPoint) iterator.next();
//				Rect rec = Imgproc.boundingRect(matOfPoint);
//					if(rec.height < 25 || rec.width < 25){
//						iterator.remove();
//					continue;
//					}
//					float aspect = (float)rec.width/(float)rec.height;
//					if(aspect < 1.0)
//						iterator.remove();
//				}
//				for(MatOfPoint mop : contours){
//					Rect rec = Imgproc.boundingRect(mop);
//					Imgproc.rectangle(matOriginal, rec.br(), rec.tl(), BLACK);
//					
//			}
////			if there is only 1 target, then we have found the target we want
//			if(contours.size() == 1){
//				Rect rec = Imgproc.boundingRect(contours.get(0));
//				y = (rec.br().y + rec.height) / 2.0;
//				y= 2.0*y;
//				/*distance = ((TOP_TARGET_HEIGHT - TOP_CAMERA_HEIGHT) / 
//						Math.tan((y * VERTICAL_FOV / 2.0 + CAMERA_ANGLE) * Math.PI / 180.0));*/
//				//distance = rec.height*rec.width; //156.5 inches = 3417.443298969072 units
//												 //135.25 inches = 4171 units 1.246
//				//distance = 156.5*3417.443298969072/distance;
//				
//				
//				distance = (16.0*HORIZONTAL_FOV)/(2.0*(double)(rec.width)*Math.tan(2.0*HORIZONTAL_FOV*(Math.PI/180.0))); //16=width in inches of target
//				System.out.println("Distance: "+distance); //should be 147
//				
//				
////				angle to target...would not rely on this
//				//targetX = rec.tl().x + rec.width / 2.0;
//				//targetX = (2.0 * (targetX / matOriginal.width())) - 1.0;
//				targetX = HORIZONTAL_FOV*(.5-(((rec.tl().x+rec.br().x)/2.0)/matOriginal.width()));
//				//azimuth = normalize360(targetX*HORIZONTAL_FOV /2.0);
//				azimuth = ((int)targetX)%360;
////				drawing info on target
//				Point center = new Point(rec.br().x-rec.width / 2.0 - 15.0,rec.br().y - rec.height / 2.0);
//				Point centerw = new Point(rec.br().x-rec.width / 2.0 - 15.0,rec.br().y - rec.height / 2.0 - 20.0);
//				Imgproc.putText(matOriginal, ""+(int)distance, center, Core.FONT_HERSHEY_PLAIN, 1, BLACK);
//				Imgproc.putText(matOriginal, ""+(int)azimuth, centerw, Core.FONT_HERSHEY_PLAIN, 1, BLACK);
//				SmartDashboard.putNumber("Distance", distance);
//				SmartDashboard.putNumber("Azimuth", azimuth);
//				average+=distance;
//				asdfasdf++;
//			}
////			output an image for debugging
//			
//			// /home/lvuser/output.png
//			Imgcodecs.imwrite(("output".concat(Integer.toString(iteration).concat(".png"))), matOriginal);
//			iteration++;
//			FrameCount++;
//		}
//		System.out.println(average/asdfasdf);
//		videoCapture.release();
//	}
//	public static double normalize360(double angle){
//		return angle%360.0;
//	}
//	@Override
//	protected void initDefaultCommand() {
//				
//	}
//	
//	/**
//	 * The point of this is to only get a bounding rectangle, nothing more. Distance CAN be shown,
//	 *  but it doesn't take a genius to figure out that this will be CPU/GPU intensive.
//	 * 
//	 * INCOMPLETE
//	 * 
//	 * @param iterations
//	 * @return bounding rectangle
//	 * 
//	 */
//	public static Rect onlyGetRect(int iterations){
//		ArrayList<MatOfPoint> contours = new ArrayList<MatOfPoint>();
//		int FrameCount = 0;
//		double ax=0,ay=0,aw=0,ah=0;
//		while(FrameCount < iterations){
//			contours.clear();
//			Imgproc.cvtColor(matOriginal,matHSV,Imgproc.COLOR_BGR2HSV);			
//			Core.inRange(matHSV, LOWER_BOUNDS, UPPER_BOUNDS, matThresh);
//			Imgproc.findContours(matThresh, contours, matHeirarchy, Imgproc.RETR_EXTERNAL, 
//					Imgproc.CHAIN_APPROX_SIMPLE);
//			for (Iterator<MatOfPoint> iterator = contours.iterator(); iterator.hasNext();) {
//				MatOfPoint matOfPoint = (MatOfPoint) iterator.next();
//				Rect rec = Imgproc.boundingRect(matOfPoint);
//					if(rec.height < 25 || rec.width < 25){
//						iterator.remove();
//					continue;
//					}
//					float aspect = (float)rec.width/(float)rec.height;
//					if(aspect < 1.0)
//						iterator.remove();
//				}
//				for(MatOfPoint mop : contours){
//					Rect rec = Imgproc.boundingRect(mop);
//					Imgproc.rectangle(matOriginal, rec.br(), rec.tl(), BLACK);
//					
//			}
//				
//			if(contours.size() == 1){
//				Rect rec = Imgproc.boundingRect(contours.get(0));
//				ax+=rec.x;
//				ay+=rec.y;
//				aw+=rec.width;
//				ah+=rec.height;
//				
//			}
//			FrameCount++;
//			
//		}
//		videoCapture.release();
//		return new Rect((int)(ax/iterations),(int)(ay/iterations),(int)(aw/iterations),(int)(ah/iterations));
//	}
//	
//	
//}
