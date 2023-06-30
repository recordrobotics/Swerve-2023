package frc.robot.control;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.Swerve;

public class SingleControl implements IControlInput {

	private XboxController _gamepad;

	private static final double maxSpeed = 1; // m/s

	private static final double maxSpin = 1; // r/s

	public SingleControl(int port) {
		_gamepad = new XboxController(port);
	}

	public double getX() {
		return _gamepad.getLeftX() * maxSpeed / Swerve.GEAR_RATIO;
	}

	public double getY() {
		return _gamepad.getLeftY() * maxSpeed / Swerve.GEAR_RATIO;
	}

	public double getSpin() {
		return _gamepad.getRightX() * maxSpin / Swerve.GEAR_RATIO;
	}
}
