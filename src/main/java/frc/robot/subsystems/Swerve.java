// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotMap;

public class Swerve extends SubsystemBase {
        // todo get constants

        // TODO change to correct motor
        private TalonFX[] speedMotors = {
                        new TalonFX(RobotMap.swerve.SPEED_MOTORS[0]),
                        new TalonFX(RobotMap.swerve.SPEED_MOTORS[1]),
                        new TalonFX(RobotMap.swerve.SPEED_MOTORS[2]),
                        new TalonFX(RobotMap.swerve.SPEED_MOTORS[3])
        };
        private TalonFX[] directionMotors = {
                        new TalonFX(RobotMap.swerve.DIRECTION_MOTORS[0]),
                        new TalonFX(RobotMap.swerve.DIRECTION_MOTORS[1]),
                        new TalonFX(RobotMap.swerve.DIRECTION_MOTORS[2]),
                        new TalonFX(RobotMap.swerve.DIRECTION_MOTORS[3])
        };
        private DutyCycleEncoder[] encoders = {
                        new DutyCycleEncoder(RobotMap.swerve.DEVICE_NUMBER[0]),
                        new DutyCycleEncoder(RobotMap.swerve.DEVICE_NUMBER[1]),
                        new DutyCycleEncoder(RobotMap.swerve.DEVICE_NUMBER[2]),
                        new DutyCycleEncoder(RobotMap.swerve.DEVICE_NUMBER[3])
        };
        private PIDController[] dPID = {
                        new PIDController(Constants.Swerve.kp, Constants.Swerve.ki, Constants.Swerve.kd),
                        new PIDController(Constants.Swerve.kp, Constants.Swerve.ki, Constants.Swerve.kd),
                        new PIDController(Constants.Swerve.kp, Constants.Swerve.ki, Constants.Swerve.kd),
                        new PIDController(Constants.Swerve.kp, Constants.Swerve.ki, Constants.Swerve.kd)
        };
        private final double moduleWidth = 0.762;
        private final double moduleLength = 0.762;

        Translation2d[] locations = {
                        new Translation2d(moduleWidth / 2, moduleLength / 2),
                        new Translation2d(moduleWidth / 2, -(moduleLength / 2)),
                        new Translation2d(-(moduleWidth / 2), moduleLength / 2),
                        new Translation2d(-(moduleWidth / 2), -(moduleLength / 2)),
        };

        SwerveDriveKinematics kinematics = new SwerveDriveKinematics(locations[0], locations[1],
                        locations[2], locations[3]);
        
        private SwerveModuleState[] MOD_TARGETS = {
                        new SwerveModuleState(),
                        new SwerveModuleState(),
                        new SwerveModuleState(),
                        new SwerveModuleState()
        };

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

        /**
         * Target Velocity and Angle
         */
        ChassisSpeeds target = new ChassisSpeeds();

        public Swerve() {
                encoders[0].setPositionOffset( 1 - 0.6255);
                encoders[1].setPositionOffset(1- 0.4168);
                encoders[2].setPositionOffset(1-0.8625);
                encoders[3].setPositionOffset(1-0.6967);

                for (int i = 0; i < encoders.length; i++) {
                        directionMotors[i].configNeutralDeadband(0.001);
                        speedMotors[i].set(ControlMode.PercentOutput, 0);
                        directionMotors[i].set(ControlMode.PercentOutput, 0);
                        directionMotors[i].setSelectedSensorPosition(encoders[i].getAbsolutePosition() * 2048);
                        dPID[i].enableContinuousInput(-0.5, 0.5);
                }

        }

        /**
         * gets current module states
         *  */ 
        public SwerveModuleState[] modState() {
                SwerveModuleState[] LFState = {
                                new SwerveModuleState(
                                                speedMotors[0].getSelectedSensorVelocity() * 10 / 2048
                                                                * (0.05 * 2 * Math.PI),
                                                new Rotation2d(
                                                                directionMotors[0].getSelectedSensorPosition() / 2048
                                                                                * 2 * Math.PI
                                                                                / Constants.Swerve.DIRECTION_GEAR_RATIO)),
                                new SwerveModuleState(
                                                speedMotors[1].getSelectedSensorVelocity() * 10 / 2048
                                                                * (0.05 * 2 * Math.PI),
                                                new Rotation2d(
                                                                directionMotors[1].getSelectedSensorPosition() / 2048
                                                                                * 2 * Math.PI
                                                                                / Constants.Swerve.DIRECTION_GEAR_RATIO)),
                                new SwerveModuleState(
                                                speedMotors[2].getSelectedSensorVelocity() * 10 / 2048
                                                                * (0.05 * 2 * Math.PI),
                                                new Rotation2d(
                                                                directionMotors[2].getSelectedSensorPosition() / 2048
                                                                                * 2 * Math.PI
                                                                                / Constants.Swerve.DIRECTION_GEAR_RATIO)),
                                new SwerveModuleState(
                                                speedMotors[3].getSelectedSensorVelocity() * 10 / 2048
                                                                * (0.05 * 2 * Math.PI),
                                                new Rotation2d(
                                                                directionMotors[3].getSelectedSensorPosition() / 2048
                                                                                * 2 * Math.PI
                                                                                / Constants.Swerve.DIRECTION_GEAR_RATIO))
                };
                return LFState;
        }

        public void setTarget(ChassisSpeeds _target) {
                target = _target;
        }

        @Override
        public void periodic() {
                MOD_TARGETS = kinematics.toSwerveModuleStates(target);
                for (int i = 0; i < MOD_TARGETS.length; i++) {
                        SmartDashboard.putNumber("ABS Encoder " + i, encoders[i].getAbsolutePosition());
                        SmartDashboard.putNumber("Relative Encoder " + i, encoders[i].get());
                        SmartDashboard.putNumber("M" + i, MOD_TARGETS[i].angle.getRotations());
                        SmartDashboard.putNumber("Direction " + i, directionMotors[i].getSelectedSensorPosition() / 2048
                                        / Constants.Swerve.DIRECTION_GEAR_RATIO);
                        // position PIDs
                        dPID[i].setSetpoint(MOD_TARGETS[i].angle.getRotations());
                        // sets speed/position of the motors
                        speedMotors[i].set(ControlMode.PercentOutput, MOD_TARGETS[i].speedMetersPerSecond);
                        directionMotors[i].set(ControlMode.PercentOutput,
                                        dPID[i].calculate(directionMotors[i].getSelectedSensorPosition() / 2048
                                                        / Constants.Swerve.DIRECTION_GEAR_RATIO));
                }
        }

        @Override
        public void simulationPeriodic() {

        }
}
