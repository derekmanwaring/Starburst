package org.usfirst.frc.team4077.robot;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.io.InputStreamReader;
import java.lang.management.ManagementFactory;
import java.net.Inet4Address;
import java.net.UnknownHostException;
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
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.SpeedController;
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

//Note to thy self. Ye of little faith giveth up not hope -Scott Dong
public class Robot extends IterativeRobot {
private static final double SPEED_FACTOR = 0.50;
//	Definitions of OBjects
	DoubleSolenoid Piston1 = new DoubleSolenoid(0, 1);
	DoubleSolenoid Piston2 = new DoubleSolenoid(2, 3);
	DoubleSolenoid Hand = new DoubleSolenoid(4, 5);
	SpeedController frontLeft;
	SpeedController rearLeft;
	SpeedController frontRight; 
	SpeedController rearRight;
	SpeedController RopeClimb = new CANTalon(5);
	Joystick Drivestick = new Joystick(0);
	Joystick Armstick = new Joystick(1);
	Timer timer = new Timer();
	RobotDrive myRobot;
	int centerX = 0;
	int centerY = 0;
	long lastTimeSeen = 0;
	int numberOfContours = 0;
	int separationDistance = 0;
	Compressor C = new Compressor(0);
	AnalogInput irSensor = new AnalogInput(0);
	    
	RobotName robotName;
	private GearHandState gearHandState = GearHandState.CLOSE;
	enum RobotName {
		STELLA,
		SUMMER
		
	
	}
	enum GearHandState {
		OPEN,
		CLOSE
	}
	enum AutoState {
		STARTLEFT,
		STARTRIGHT,
		STARTCENTER,
		VISION,
		GEARDROP,
		BACKUP
	}
	enum StartPosition {
		LEFT,
		RIGHT,
		CENTER
	}
	private AutoState autoState = AutoState.VISION;
	private StartPosition startPosition = StartPosition.LEFT;
	private double gearDropFinished;
	private double visionFinished;
	private UsbCamera camera;
	private VisionThread visionThread;
	private Thread visionThread2;

	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		camera = CameraServer.getInstance().startAutomaticCapture();
		camera.setResolution(640, 480);
		C.setClosedLoopControl(true);
		motorSetup();
	
		
		
		
		
		
//		Camera Section and Vision Tracking	

		
	
		
	}
	private void motorSetup() {
		File cpuInfoFile = new File("/etc/RobotName");
		String line = null;
		try {
			FileInputStream cpuFileInputStream = new FileInputStream(cpuInfoFile);
			InputStreamReader cpuFileInputStreamReader = new InputStreamReader(cpuFileInputStream);
			BufferedReader cpuFileBufferedReader = new BufferedReader(cpuFileInputStreamReader);
			line = cpuFileBufferedReader.readLine();
		} catch (FileNotFoundException e) {
			System.out.println("Robotname file not found");
		} catch (IOException e) {
			System.out.println("Error reading robotname file");
			e.printStackTrace();
		}
		System.out.println("Read RobotName from robotname:" + line);
		if (line != null) {
			if (line.equals("Stella")) {
				robotName = RobotName.STELLA;
				frontLeft = new ScaledCANTalon(2, SPEED_FACTOR);
				rearLeft = new ScaledCANTalon(1, SPEED_FACTOR);
				frontRight = new ScaledCANTalon(3, SPEED_FACTOR);
				rearRight = new ScaledCANTalon(4, SPEED_FACTOR);
				
				frontLeft.setInverted(true);
				rearLeft.setInverted(true);
				frontRight.setInverted(true);
				rearRight.setInverted(true);
				

			}else if (line.equals("Summer")) {
				robotName = RobotName.SUMMER;
				frontLeft = new ScaledCANTalon(3, SPEED_FACTOR);
				rearLeft = new ScaledCANTalon(4, SPEED_FACTOR);
				frontRight = new ScaledCANTalon(2, SPEED_FACTOR);
				rearRight = new ScaledCANTalon(1, SPEED_FACTOR);
			}else{
				System.out.println("I don't know robotname" + line);
			}
			myRobot = new RobotDrive(frontLeft, rearLeft, frontRight, rearRight);
		}
	}
	private void visionTrackingCamera() {
		// TODO Auto-generated method stub
		camera.setResolution(320, 240);
		Object imgLock = new Object();
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
				Imgproc.circle(mat, new Point(centerX,centerY), 20, new Scalar(255,0,0), 3);
				if (autoState == AutoState.VISION){
					System.out.println(String.format("Last seen %d seconds ago, Number: %1d, Distance: %3d, X: %3d, Y:%3d",
							(System.currentTimeMillis() - lastTimeSeen) / 1000,
							numberOfContours, separationDistance, centerX, centerY));
				}
						
				
		
				// Give the output stream a new image to display
				outputStream.putFrame(mat);
			}
		});
		visionThread2.setDaemon(true);
		visionThread2.start();
		

	}
	/**
	 * This function is run once each time the robot enters autonomous mode
	 */
	@Override
	public void autonomousInit() {
		visionTrackingCamera();
//		Change line below to change code for position of robot LEFt/RIGHT/CENTER
		startPosition = StartPosition.RIGHT;
		
		timer.reset();
		if (startPosition == StartPosition.LEFT){
			autoState = AutoState.STARTLEFT;
		}else if (startPosition == StartPosition.RIGHT){
			autoState = AutoState.STARTRIGHT;
			
		}else if (startPosition == StartPosition.CENTER){
			autoState = AutoState.STARTCENTER;
		}
		Piston1.set(DoubleSolenoid.Value.kForward);
		Piston2.set(DoubleSolenoid.Value.kReverse);
		timer.start();
	}

	/**
	 * This function is called periodically during autonomous
	 */
	@Override
	public void autonomousPeriodic() {
//		if (timer.get() < 1.0) {
//			myRobot.drive(-0.6, 0.0);
//		}else if (timer.get() <1.5) {
//			myRobot.tankDrive(-0.4, 0.4);
		switch (autoState) {
		case STARTCENTER:
			autoState = AutoState.VISION;
			break;
		case STARTLEFT:
			if (timer.get() < 2.0) {
				myRobot.drive(-0.6, 0.0);
				}else if (timer.get() < 3.0){
					myRobot.tankDrive(-0.65, 0.65);
				}else{
					System.out.println("Changing autostate to vision");
					autoState = AutoState.VISION;
				}
			break;
		case STARTRIGHT:
			if (timer.get() < 2.0) {
				myRobot.drive(-0.6, 0.0);
				}else if (timer.get() < 3.0){
					myRobot.tankDrive(0.65, -0.65);
				}else{
					autoState = AutoState.VISION;
				}
			break;
		case VISION:
			visionDrive();
			break;
		case GEARDROP:
			System.out.println("Changing Autostate to Backup");
			gearHandState = GearHandState.OPEN;
			if (timer.get() - visionFinished < 0.5) {
				autoState = AutoState.BACKUP;
				gearDropFinished = timer.get();
			}
			break;
		case BACKUP:
			if (timer.get() - gearDropFinished < 1.0) {
				myRobot.drive(0.5, 0.0);
			}
			break;
		}
			
		
		
		setGearHandSolenoid();
		
	}
	private void visionDrive() {

		
		if (separationDistance < 150) {
			double curve;
			curve = (((double) centerX) - 160.0) / 160.0;
			myRobot.drive(-0.4, curve);
		}
		if (timer.get() > 10){
			System.out.println("Changing auto to geardrop");
			myRobot.drive (0.0,0.0);
			autoState = AutoState.GEARDROP;
			visionFinished = timer.get();
		}


		
	}

	/**
	 * This function is called once each time the robot enters tele-operated
	 * mode
	 */
	@Override
	public void teleopInit() {
		if (visionThread != null) {
			visionThread.interrupt();
			visionThread2.interrupt();
		}
		camera.setResolution(640, 480);
	}

	/**
	 * This function is called periodically during operator control
	 */
	@Override
	public void teleopPeriodic() {
//		Arcade Drive for Robot
		myRobot.arcadeDrive(Drivestick);
//		myRobot.tankDrive(Drivestick.getRawAxis(1), Drivestick.getRawAxis(5));;
	System.out.println("IR Value" + irSensor.getValue() );

//		Solenoids Control
		if (Drivestick.getRawButton(3) || Armstick.getRawButton(3)) {
			Piston1.set(DoubleSolenoid.Value.kReverse);
			Piston2.set(DoubleSolenoid.Value.kReverse);
			
		}

		if (Drivestick.getRawButton(4)|| Armstick.getRawButton(4)) {
			Piston1.set(DoubleSolenoid.Value.kForward);
			Piston2.set(DoubleSolenoid.Value.kReverse);
			
		}
		if (Drivestick.getRawButton(2)|| Armstick.getRawButton(2)) {
			Piston1.set(DoubleSolenoid.Value.kForward);
			Piston2.set(DoubleSolenoid.Value.kForward);
		}
		if (Drivestick.getRawButton(6) || Armstick.getRawButton(6)) {
			gearHandState = GearHandState.OPEN;
		} else{
			gearHandState = GearHandState.CLOSE;
		}
	
		if (Drivestick.getRawButton(5) || Armstick.getRawButton(5)) {
			RopeClimb.set(-0.75);
	}else{
		RopeClimb.set(0.0);
	}
	
	
	
		
		
	
		setGearHandSolenoid();
			
	}
	private void setGearHandSolenoid() {
		
		if (gearHandState  == GearHandState.OPEN) {
			Hand.set(DoubleSolenoid.Value.kForward);
		}
		else if (gearHandState == GearHandState.CLOSE){
				Hand.set(DoubleSolenoid.Value.kReverse);
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