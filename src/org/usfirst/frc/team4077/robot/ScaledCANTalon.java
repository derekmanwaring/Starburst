package org.usfirst.frc.team4077.robot;

import com.ctre.CANTalon;

import edu.wpi.first.wpilibj.SpeedController;

public class ScaledCANTalon implements SpeedController {
	private SpeedController wrappedTalon;
	private double scalingFactor;
	
	public ScaledCANTalon(int CANChannel, double speedFactor) {
		// TODO Auto-generated constructor stub
		wrappedTalon = new CANTalon(CANChannel);
		scalingFactor = speedFactor;
	}

	@Override
	public void pidWrite(double output) {
		// TODO Auto-generated method stub
		wrappedTalon.pidWrite(output);
	}

	@Override
	public double get() {
		// TODO Auto-generated method stub
		return wrappedTalon.get() / scalingFactor;
	}

	@Override
	public void set(double speed) {
		// TODO Auto-generated method stub
		wrappedTalon.set(speed * scalingFactor);
		
	}

	@Override
	public void setInverted(boolean isInverted) {
		// TODO Auto-generated method stub
		wrappedTalon.setInverted(isInverted);
	}

	@Override
	public boolean getInverted() {
		// TODO Auto-generated method stub
		return wrappedTalon.getInverted();
				
	}

	@Override
	public void disable() {
		// TODO Auto-generated method stub
		wrappedTalon.disable();

	}

	@Override
	public void stopMotor() {
		// TODO Auto-generated method stub
		wrappedTalon.stopMotor();
	}

}
