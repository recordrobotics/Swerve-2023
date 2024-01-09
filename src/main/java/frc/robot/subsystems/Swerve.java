// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotMap;

/**
 *  TODO: Add descriptive class comment here.
 */
public class Swerve extends SubsystemBase {
        // TODO: This should have a comment explaining how these were obtained.
        private final double moduleWidth = 0.762;
        private final double moduleLength = 0.762;

        private TalonFX[] speedMotors = new TalonFX[Constants.Swerve.NUM_WHEELS];
        private TalonFX[] directionMotors = new TalonFX[Constants.Swerve.NUM_WHEELS];
        private DutyCycleEncoder[] encoders = new DutyCycleEncoder[Constants.Swerve.NUM_WHEELS];
        private PIDController[] dPID = new PIDController[Constants.Swerve.NUM_WHEELS];
        private SwerveModuleState[] modTargets = new SwerveModuleState[Constants.Swerve.NUM_WHEELS];

        // TODO: This needs a comment explaining what this is doing
        Translation2d[] locations = {
                new Translation2d(moduleWidth / 2, moduleLength / 2),
                new Translation2d(moduleWidth / 2, -(moduleLength / 2)),
                new Translation2d(-(moduleWidth / 2), moduleLength / 2),
                new Translation2d(-(moduleWidth / 2), -(moduleLength / 2)),
        };

        SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
                locations[0], locations[1], locations[2], locations[3]);

        /*
         * private static final SwerveModulePosition[] startPos = {
         * new SwerveModulePosition(),
         * new SwerveModulePosition(),
         * new SwerveModulePosition(),
         * new SwerveModulePosition()
         * };
         */
        // SwerveDriveOdometry m_Odometry = new SwerveDriveOdometry(kinematics,
        // _nav.getRotation2d(), startPos);

        // Target Velocity and Angle
        ChassisSpeeds target = new ChassisSpeeds();

        /**
         * Constructor.  Creates the various subsystems this depends on, and sets their initial states.
         */
        public Swerve() {
                // TODO: Should have a comment explaining what it is.
                for (int i = 0; i < Constants.Swerve.NUM_WHEELS; i++) {
                        // motors
                        speedMotors[i] = new TalonFX(RobotMap.swerve.SPEED_MOTORS[i]);
                        directionMotors[i] = new TalonFX(RobotMap.swerve.DIRECTION_MOTORS[i]);
                        // absolute encoders
                        encoders[i] = new DutyCycleEncoder(RobotMap.swerve.DEVICE_NUMBER[i]);
                        // PID
                        dPID[i] = new PIDController(Constants.Swerve.kp, Constants.Swerve.ki, Constants.Swerve.kd);
                        modTargets[i] = new SwerveModuleState();
                }

                // encoder offsets
                // TODO: This needs a comment explaining where these constants came from,
                // and how to regenerate them if needed.
                encoders[0].setPositionOffset(1 - 0.6255);
                encoders[1].setPositionOffset(1 - 0.4168);
                encoders[2].setPositionOffset(1 - 0.8625);
                encoders[3].setPositionOffset(1 - 0.6967);

                // motor + PID settings
                for (int i = 0; i < Constants.Swerve.NUM_WHEELS; i++) {
                        directionMotors[i].configNeutralDeadband(0.001);
                        speedMotors[i].set(ControlMode.PercentOutput, 0);
                        directionMotors[i].set(ControlMode.PercentOutput, 0);
                        directionMotors[i].setSelectedSensorPosition(encoders[i].getAbsolutePosition() * 2048);
                        dPID[i].enableContinuousInput(-0.5, 0.5);
                }
        }

        /**
         * gets current module states
         */
        public SwerveModuleState[] modState() {
                SwerveModuleState[] LFState = {
                        new SwerveModuleState(
                                // TODO: This conversion functionality should be refactored into a 
                                // utility method that can be shared, and the logic / constants documented.
                                speedMotors[0].getSelectedSensorVelocity() * 10 / 2048
                                                * (0.05 * 2 * Math.PI),
                                // TODO: This conversion functionality should be refactored into a 
                                // utility method that can be shared, and the logic / constants documented.
                                new Rotation2d(directionMotors[0].getSelectedSensorPosition() / 2048
                                                        * 2 * Math.PI
                                                        / Constants.Swerve.DIRECTION_GEAR_RATIO)),
                        new SwerveModuleState(
                                speedMotors[1].getSelectedSensorVelocity() * 10 / 2048
                                                * (0.05 * 2 * Math.PI),
                                new Rotation2d(directionMotors[1].getSelectedSensorPosition() / 2048
                                                        * 2 * Math.PI
                                                        / Constants.Swerve.DIRECTION_GEAR_RATIO)),
                        new SwerveModuleState(
                                speedMotors[2].getSelectedSensorVelocity() * 10 / 2048
                                                * (0.05 * 2 * Math.PI),
                                new Rotation2d(directionMotors[2].getSelectedSensorPosition() / 2048
                                                        * 2 * Math.PI
                                                        / Constants.Swerve.DIRECTION_GEAR_RATIO)),
                        new SwerveModuleState(
                                speedMotors[3].getSelectedSensorVelocity() * 10 / 2048
                                                * (0.05 * 2 * Math.PI),
                                new Rotation2d(directionMotors[3].getSelectedSensorPosition() / 2048
                                                        * 2 * Math.PI
                                                        / Constants.Swerve.DIRECTION_GEAR_RATIO))
                };
                return LFState;
        }

        // run in manualswerve to get target speed
        public void setTarget(ChassisSpeeds _target) {
                target = _target;
        }

        @Override
        public void periodic() {
                // converts target speeds to swerve module angle and rotations
                modTargets = kinematics.toSwerveModuleStates(target);
                for (int i = 0; i < Constants.Swerve.NUM_WHEELS; i++) {
                        SmartDashboard.putNumber("ABS Encoder " + i, encoders[i].getAbsolutePosition());
                        SmartDashboard.putNumber("Relative Encoder " + i, encoders[i].get());
                        // TODO: Instead of "M#", this should have a clearer label
                        SmartDashboard.putNumber("M" + i, modTargets[i].angle.getRotations());
                        SmartDashboard.putNumber("Direction " + i, directionMotors[i].getSelectedSensorPosition() / 2048
                                        / Constants.Swerve.DIRECTION_GEAR_RATIO);

                        // position PIDs
                        dPID[i].setSetpoint(modTargets[i].angle.getRotations());

                        // sets speed/position of the motors
                        speedMotors[i].set(ControlMode.PercentOutput, modTargets[i].speedMetersPerSecond);
                        directionMotors[i].set(ControlMode.PercentOutput,
                                        // TODO: This should also be factored out into a utility function.
                                        dPID[i].calculate(directionMotors[i].getSelectedSensorPosition() / 2048
                                                        / Constants.Swerve.DIRECTION_GEAR_RATIO));
                }
        }

        @Override
        public void simulationPeriodic() { }
}
