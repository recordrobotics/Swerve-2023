package frc.robot.control;

import edu.wpi.first.wpilibj.Joystick;

public class SingleControl implements IControlInput {

	private Joystick _gamepad;

	private static final double speedModifier = 0.5;

	public SingleControl(int port) {
		_gamepad = new Joystick(port);
	}

	public double getX() {
		return _gamepad.getX() * speedModifier;
	}

	public double getY() {
		return _gamepad.getY() * speedModifier;
	}

	public boolean setSpin() {
		return _gamepad.getTrigger();
	}
}
