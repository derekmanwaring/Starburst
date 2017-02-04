package org.usfirst.frc.team4077.robot;

import org.opencv.core.KeyPoint;
import org.opencv.core.Mat;
import org.opencv.core.MatOfKeyPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import com.ctre.CANTalon;
import org.usfirst.frc.team4077.robot.GripPipeline;
import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;
import com.ctre.CANTalon;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.vision.VisionRunner;
import edu.wpi.first.wpilibj.vision.VisionThread;
import edu.wpi.first.wpilibj.DoubleSolenoid;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {
//	Definitions of OBjects
	DoubleSolenoid DoubleSolenoide = new DoubleSolenoid(0, 1);
	CANTalon frontLeft = new CANTalon(2);
	CANTalon rearLeft = new CANTalon(1);
	CANTalon frontRight = new CANTalon(3);
	CANTalon rearRight= new CANTalon(4);
	Joystick stick = new Joystick(0);
	Timer timer = new Timer();
	RobotDrive myRobot = new RobotDrive(frontLeft, rearLeft, frontRight, rearRight);
	

	private VisionThread visionThread;
	private double centerX = 0.0;
	private final Object imgLock = new Object();


	    
	

	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		frontLeft.setInverted(true);
		rearLeft.setInverted(true);
		frontRight.setInverted(true);
		rearRight.setInverted(true);
		
//		Camera Section and Vision Tracking	
		UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
		// Set the resolution
		camera.setResolution(640, 480);
		
		// Get a CvSink. This will capture Mats from the camera
		CvSink cvSink = CameraServer.getInstance().getVideo();
		// Setup a CvSource. This will send images back to the Dashboard
		CvSource outputStream = CameraServer.getInstance().putVideo("Circle", 640, 480);

	 	Mat mat = new Mat();
		    visionThread = new VisionThread(camera, new GripPipeline(), pipeline -> {
				Imgproc.circle(mat, new Point(0.0, 0.0), 40, new Scalar(255, 255, 255));
				outputStream.putFrame(mat);
		   
//		    	outputStream.putFrame(mat);
//				if (cvSink.grabFrame(mat) == 0) {
//					// Send the output the error.
//					outputStream.notifyError(cvSink.getError());
//					// skip the rest of the current iteration
//					return;
//				}
//		        MatOfKeyPoint findBlobsOutput = pipeline.findBlobsOutput();
//				if (findBlobsOutput.empty()) {
//					KeyPoint theFirstKeyPoint = findBlobsOutput.toList().get(0);
//					Imgproc.circle(mat, theFirstKeyPoint.pt, (int)(theFirstKeyPoint.size / 2.0f), new Scalar(255, 255, 255));
//		            synchronized (imgLock) {
//		                centerX = theFirstKeyPoint.pt.x;
//		                System.out.println("CenterX" + centerX);
//		            }
//		        }
		    });
		    visionThread.start();
		        
		

		
	}
	/**
	 * This function is run once each time the robot enters autonomous mode
	 */
	@Override
	public void autonomousInit() {
		timer.reset();
		timer.start();
	}

	/**
	 * This function is called periodically during autonomous
	 */
	@Override
	public void autonomousPeriodic() {
		// Drive for 2 seconds
		if (timer.get() < 2.0) {
			myRobot.drive(-0.5, 0.0); // drive forwards half speed
		} else if (timer.get() <3.0) {
			myRobot.tankDrive(0.71, -0.71);
			
		} else if (timer.get() <5.0) {
			myRobot.drive(-0.5, 0.0);
		} else if (timer.get() <6.0){
		myRobot.tankDrive(0.71, -0.71);
		
		} else if (timer.get() <8.0) {
			myRobot.drive(-0.5, 0.0);
		} else if (timer.get() <9.0) {
			myRobot.tankDrive(0.71, -0.71);
			
		} else if (timer.get() <11.0) {
			myRobot.drive(-0.5, 0.0);
		} else if (timer.get() <12.0) {
			myRobot.tankDrive(0.71, -0.71);
		} else {	
			myRobot.drive(0.0, 0.0); // stop robot
		}
	}

	/**
	 * This function is called once each time the robot enters tele-operated
	 * mode
	 */
	@Override
	public void teleopInit() {
	}

	/**
	 * This function is called periodically during operator control
	 */
	@Override
	public void teleopPeriodic() {
//		Arcade Drive for Robot
		myRobot.arcadeDrive(stick);
		System.out.println("x-axis; " + stick.getX());
		System.out.println("y-axis; " + stick.getX());
		
		if(stick.getRawButton(7))
			 DoubleSolenoide.set(DoubleSolenoid.Value.kForward);
		else DoubleSolenoide.set(DoubleSolenoid.Value.kOff);
		System.out.println("DoubleSolenoid;"+ DoubleSolenoide.get());
//			DoubleSolenoide.set(DoubleSolenoid.Value.kReverse);


	}

	/**
	 * This function is called periodically during test mode
	 */
	@Override
	public void testPeriodic() {
		LiveWindow.run();
	}
}