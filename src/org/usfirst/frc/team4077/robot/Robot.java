package org.usfirst.frc.team4077.robot;

import java.util.ArrayList;
import java.util.Date;

import org.opencv.core.KeyPoint;
import org.opencv.core.Mat;
import org.opencv.core.MatOfKeyPoint;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import com.ctre.CANTalon;
import org.usfirst.frc.team4077.robot.GripPipeline;
import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.Compressor;
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
	DoubleSolenoid Piston1 = new DoubleSolenoid(0, 1);
	DoubleSolenoid Piston2 = new DoubleSolenoid(2, 3);
	DoubleSolenoid Hand = new DoubleSolenoid(4, 5);
	CANTalon frontLeft = new CANTalon(3);
	CANTalon rearLeft = new CANTalon(4);
	CANTalon frontRight = new CANTalon(2);
	CANTalon rearRight= new CANTalon(1);
	CANTalon RopeClimb = new CANTalon(5);
	Joystick stick = new Joystick(0);
	Timer timer = new Timer();
	RobotDrive myRobot = new RobotDrive(frontLeft, rearLeft, frontRight, rearRight);
	int centerX = 0;
	int centerY = 0;
	long lastTimeSeen = 0;
	int numberOfContours = 0;
	int separationDistance = 0;
	Compressor c = new Compressor(0);
	    
	

	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
//		frontLeft.setInverted(true);
//		rearLeft.setInverted(true);
//		frontRight.setInverted(true);
//		rearRight.setInverted(true);
		
		
		
		
		
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
			ArrayList<MatOfPoint> filterContoursOutput = pipeline.filterContoursOutput();
			numberOfContours = filterContoursOutput.size();
			if (numberOfContours == 2){
				Rect r1= Imgproc.boundingRect(filterContoursOutput.get(0));
				Rect r2= Imgproc.boundingRect(filterContoursOutput.get(1));
				int	centerXr1 = r1.x + (r1.width/2);
				int	centerYr1 = r1.y + (r1.height/2);
				int	centerXr2 = r2.x + (r2.width/2);
				int	centerYr2 = r2.y + (r2.height/2);
				int centerXBiggerValue = 0;
				int centerXSmallerValue = 0;
				if (centerXr1 < centerXr2){
					centerXBiggerValue = centerXr2;
					centerXSmallerValue = centerXr1;
				}else{
					centerXBiggerValue = centerXr1;
					centerXSmallerValue = centerXr2;
				}
				 separationDistance = centerXBiggerValue - centerXSmallerValue;
				 if (separationDistance < 160){
					 lastTimeSeen = System.currentTimeMillis();
					 centerX = separationDistance/2+centerXSmallerValue;
					 centerY = centerYr1;
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
				Date lastSeenAsDate = new Date(lastTimeSeen);
				System.out.println(String.format("Last seen at %2d:%2d, Number: %1d, Distance: %3d, X: %3d, Y:%3d",
						lastSeenAsDate.getMinutes(), lastSeenAsDate.getSeconds(),
						numberOfContours, separationDistance, centerX, centerY));
				
		
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
		if (lastTimeSeen < (System.currentTimeMillis() - 1000)){
			myRobot.drive (0.0,0.0);
			return;
		}
		if (separationDistance < 145) {
			myRobot.drive(-0.25, 0.0);
		}else{
			myRobot.drive (0.0,0.0);
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
		c.setClosedLoopControl(true);
		}
		if (stick.getRawButton(6)){
		c.setClosedLoopControl(false);
		}

	
		if (stick.getRawButton(3)) {
			Piston1.set(DoubleSolenoid.Value.kReverse);
			Piston2.set(DoubleSolenoid.Value.kReverse);
			
		}

		if (stick.getRawButton(4)) {
			Piston1.set(DoubleSolenoid.Value.kForward);
			Piston2.set(DoubleSolenoid.Value.kReverse);
			
		}
		if (stick.getRawButton(2)) {
			Piston1.set(DoubleSolenoid.Value.kForward);
			Piston2.set(DoubleSolenoid.Value.kForward);
		}
		if (stick.getRawButton(1)) {
			Hand.set(DoubleSolenoid.Value.kForward);
		}else{
			Hand.set(DoubleSolenoid.Value.kReverse);
		}
		if (stick.getRawButton(8)) {
			Piston1.set(DoubleSolenoid.Value.kOff);
			Piston2.set(DoubleSolenoid.Value.kOff);
			Hand.set(DoubleSolenoid.Value.kOff);
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