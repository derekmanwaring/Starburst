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
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
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
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {
//	Definitions of OBjects
	DoubleSolenoid DS1 = new DoubleSolenoid(0, 1);
	DoubleSolenoid DS2 = new DoubleSolenoid(2, 3);
	DoubleSolenoid DS3 = new DoubleSolenoid(4, 5);
	CANTalon frontLeft = new CANTalon(2);
	CANTalon rearLeft = new CANTalon(1);
	CANTalon frontRight = new CANTalon(3);
	CANTalon rearRight= new CANTalon(4);
	Joystick stick = new Joystick(0);
	Timer timer = new Timer();
	RobotDrive myRobot = new RobotDrive(frontLeft, rearLeft, frontRight, rearRight);
	double centerX = 0.0;
	double centerY = 0.0;

	    
	

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
//		defaultCamera();
		
		visionTrackingCamera();
		
	}
	private void visionTrackingCamera() {
		// TODO Auto-generated method stub
		VisionThread visionThread;
		Object imgLock = new Object();
		UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
		camera.setResolution(320, 240);
		visionThread = new VisionThread(camera, new GripPipeline(), pipeline -> {
			if (!pipeline.filterContoursOutput().isEmpty()){
				Rect r= Imgproc.boundingRect(pipeline.filterContoursOutput().get(0));
				synchronized (imgLock){
					centerX = r.x + (r.width/2);
					centerY = r.y -(r.height/2);
					
				}
			}
		});
		visionThread.start();
		Thread visionThread2;
		visionThread2 = new Thread(() -> {
		
			// Get a CvSink. This will capture Mats from the camera
			CvSink cvSink = CameraServer.getInstance().getVideo();
			// Setup a CvSource. This will send images back to the Dashboard
			CvSource outputStream = CameraServer.getInstance().putVideo("Target", 320, 240);

			// Mats are very memory expensive. Lets reuse this Mat.
			Mat mat = new Mat();

			// This cannot be 'true'. The program will never exit if it is. This
			// lets the robot stop this thread when restarting robot code or
			// deploying.
			while (!Thread.interrupted()) {
				// Tell the CvSink to grab a frame from the camera and put it
				// in the source mat.  If there is an error notify the output.
				if (cvSink.grabFrame(mat) == 0) {
					// Send the output the error.
					outputStream.notifyError(cvSink.getError());
					// skip the rest of the current iteration
					continue;
				}
				Imgproc.circle(mat, new Point(centerX,centerY), 20, new Scalar(0,255,0), 3);
				
		
				// Give the output stream a new image to display
				outputStream.putFrame(mat);
			}
		});
		visionThread2.setDaemon(true);
		visionThread2.start();
		
	}
	private void defaultCamera() {
		UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
		// Set the resolution
		camera.setResolution(640, 480);
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
	
		if (stick.getRawButton(5)) {
			DS1.set(DoubleSolenoid.Value.kForward);
			
		} else {
			DS1.set(DoubleSolenoid.Value.kOff);
		
		}

		if (stick.getRawButton(6)) {
			DS2.set(DoubleSolenoid.Value.kForward);
			
		} else {
			DS2.set(DoubleSolenoid.Value.kOff);
		
		}

		if (stick.getRawButton(1)) {
			DS3.set(DoubleSolenoid.Value.kForward);
			
		} else {
			DS3.set(DoubleSolenoid.Value.kOff);
		
		}
		
		
		
		

	}

	/**
	 * This function is called periodically during test mode
	 */
	@Override
	public void testPeriodic() {
		LiveWindow.run();
	}
}