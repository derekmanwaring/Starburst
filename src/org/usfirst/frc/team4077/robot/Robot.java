package org.usfirst.frc.team4077.robot;

import com.ctre.CANTalon;

import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {
	
	CANTalon frontLeft = new CANTalon(3);
	CANTalon rearLeft = new CANTalon(4);
	CANTalon frontRight = new CANTalon(1);
	CANTalon rearRight= new CANTalon(2);
	Joystick left, right; 
	Timer timer = new Timer();
	RobotDrive myRobot = new RobotDrive(frontLeft, rearLeft, frontRight, rearRight);

	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		left = new Joystick(1);
		right = new Joystick(0);
		CameraServer.getInstance().startAutomaticCapture();
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
			myRobot.tankDrive(0.73, -0.73);
			
		} else if (timer.get() <5.0) {
			myRobot.drive(-0.5, 0.0);
		} else if (timer.get() <6.0){
		myRobot.tankDrive(0.73, -0.73);
		
		} else if (timer.get() <8.0) {
			myRobot.drive(-0.5, 0.0);
		} else if (timer.get() <9.0) {
			myRobot.tankDrive(0.73, -0.73);
			
		} else if (timer.get() <11.0) {
			myRobot.drive(-0.5, 0.0);
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
		myRobot.tankDrive(left, right);

	}

	/**
	 * This function is called periodically during test mode
	 */
	@Override
	public void testPeriodic() {
		LiveWindow.run();
	}
}
 