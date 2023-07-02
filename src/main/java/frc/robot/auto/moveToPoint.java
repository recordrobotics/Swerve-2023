package frc.robot.auto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;

public class moveToPoint extends CommandBase {
	private Swerve _swerve;
	private double _speed;
	private double _y;

	private PIDController _spinPID = new PIDController(S_KP, S_KI, S_KD);;
	private PIDController _xPID = new PIDController(XY_KP, XY_KI, XY_KD);;
	private PIDController _yPID = new PIDController(XY_KP, XY_KI, XY_KD);;

	private static final double S_KP = 0.025;
	private static final double S_KI = 0.005;
	private static final double S_KD = 0;

	private static final double XY_KP = 0.025;
	private static final double XY_KI = 0.005;
	private static final double XY_KD = 0;

	private Rotation2d _rot;
	private static final double tolerance = 0.1;

	public moveToPoint(Swerve swerve, double speed, double x, double y, Rotation2d rot) {
		if (speed <= 0) {
			throw new IllegalArgumentException("Speed must be positive");
		}
		if (swerve == null) {
			throw new IllegalArgumentException("Swerve is null");
		}
		_swerve = swerve;
		_speed = speed;

		_spinPID.setSetpoint(rot.getRadians());
		_xPID.setSetpoint(x);
		_yPID.setSetpoint(y);
	}

	/**
	 * Open/close claw
	 */
	@Override

	public void initialize() {
		_swerve.setTarget(new ChassisSpeeds(
				Math.min(Math.abs(_xPID.calculate(_swerve.getX())), _speed)
						* Math.signum(_xPID.calculate(_swerve.getX())),
				Math.min(Math.abs(_yPID.calculate(_swerve.getY())), _speed)
						* Math.signum(_xPID.calculate(_swerve.getY())),
				Math.min(Math.abs(_yPID.calculate(_swerve.getY())), _speed)
						* Math.signum(_xPID.calculate(_swerve.getY()))));
	}

	/**
	 * Finished when opened or closed
	 */
	@Override
	public boolean isFinished() {
		return _x < tolerance && _x > -tolerance && _y < tolerance && _y > -tolerance && _rot.getRadians() < tolerance
				&& _y > -tolerance;
	}

	public void end(boolean interrupted) {
		_swerve.setTarget(new ChassisSpeeds(0, 0, 0));
	}
}