package frc.robot.control;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.XboxController;

public class SingleControl implements IControlInput {

	private XboxController _gamepad;

	public Rotation2d rotation = new Rotation2d(0);

	public SingleControl(int port) {
		_gamepad = new XboxController(port);
	}

	public ChassisSpeeds getSwerve() {
		return new ChassisSpeeds(_gamepad.getLeftX(), _gamepad.getLeftY(), 0);
	}
}
