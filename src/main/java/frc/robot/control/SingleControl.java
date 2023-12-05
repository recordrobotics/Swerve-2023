package frc.robot.control;

import edu.wpi.first.wpilibj.Joystick;

public class SingleControl implements IControlInput {

	private Joystick _gamepad;

	private static final double maxSpeed = 1; // m/s

	private static final double maxSpin = 1; // r/s

	public SingleControl(int port) {
		_gamepad = new Joystick(port);
	}

	public double getX() {
		return _gamepad.getX() * maxSpeed;
	}

	public double getY() {
		return _gamepad.getY() * maxSpeed;
	}

	public double getSpin() {
		return _gamepad.getTwist() * maxSpin;
	}
}
