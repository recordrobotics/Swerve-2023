package frc.robot.control;

import edu.wpi.first.math.kinematics.ChassisSpeeds;

/**
 * Specifies all control inputs needed for the robot
 */
public interface IControlInput {
	ChassisSpeeds getSwerve();
}
