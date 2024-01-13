// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotMap;

public class Swerve extends SubsystemBase {

        private final double moduleWidth = 0.762;
        private final double moduleLength = 0.762;
        private final double ABS_AT_ZERO[] = {0.411, 0.126, 0.864, 0.194};
        private final int numMotors = Constants.Swerve.NUM_SWERVE_MODS;
        private TalonFX[] speedMotors = new TalonFX[numMotors];
        private TalonFX[] directionMotors = new TalonFX[numMotors];
        private DutyCycleEncoder[] encoders = new DutyCycleEncoder[numMotors];
        private PIDController[] dPID = new PIDController[numMotors];
        private SwerveModuleState[] MOD_TARGETS = new SwerveModuleState[numMotors];
        private AHRS _nav = new AHRS(edu.wpi.first.wpilibj.I2C.Port.kMXP);
        private double compassOffset;

        Translation2d[] locations = {
                        new Translation2d(moduleWidth / 2, moduleLength / 2),
                        new Translation2d(moduleWidth / 2, -(moduleLength / 2)),
                        new Translation2d(-(moduleWidth / 2), moduleLength / 2),
                        new Translation2d(-(moduleWidth / 2), -(moduleLength / 2)),
        };

        SwerveDriveKinematics kinematics = new SwerveDriveKinematics(locations[0], locations[1],
                        locations[2], locations[3]);

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

                compassOffset = _nav.getCompassHeading();

                // Init Motors
                for (int i = 0; i < numMotors; i++) {
                        // motors
                        speedMotors[i] = new TalonFX(RobotMap.swerve.SPEED_MOTORS[i]);
                        directionMotors[i] = new TalonFX(RobotMap.swerve.DIRECTION_MOTORS[i]);
                        // absolute encoders
                        encoders[i] = new DutyCycleEncoder(RobotMap.swerve.DEVICE_NUMBER[i]);
                        // PID
                        dPID[i] = new PIDController(Constants.Swerve.kp, Constants.Swerve.ki, Constants.Swerve.kd);
                        MOD_TARGETS[i] = new SwerveModuleState();
                }

                Timer.delay(5);
                // motor + PID settings
                for (int i = 0; i < numMotors; i++) {
                        SmartDashboard.putNumber("Init Abs" + i, encoders[i].getAbsolutePosition());
                        directionMotors[i].configNeutralDeadband(0.001);
                        speedMotors[i].set(ControlMode.PercentOutput, 0);
                        directionMotors[i].set(ControlMode.PercentOutput, 0);
                        final double curAbsPos = getOffsetAbs(i);
                        final double curRelPos = -curAbsPos * Constants.Swerve.RELATIVE_ENCODER_RATIO
                                        * Constants.Swerve.DIRECTION_GEAR_RATIO;
                        directionMotors[i].setSelectedSensorPosition(curRelPos);
                        dPID[i].enableContinuousInput(-0.5, 0.5);
                }

        }

        /**
         * 
         * @param encoderIndex index of encoder in array
         * @return the absolute position of the encoder relative to our our robots zero
         */
        private double getOffsetAbs(int encoderIndex) {
                return (encoders[encoderIndex].getAbsolutePosition() - ABS_AT_ZERO[encoderIndex] + 1) % 1;
        }

        public Rotation2d getAngle() {
                return new Rotation2d((_nav.getCompassHeading() - compassOffset) / 180 * Math.PI);
        }

        /**
         * 
         * @param motorNum index of motor in array
         * @return Relative encoder in rotations
         */
        private double getRelInRotations(int motorNum) {
                return (directionMotors[motorNum].getSelectedSensorPosition() / Constants.Swerve.RELATIVE_ENCODER_RATIO)
                                / Constants.Swerve.DIRECTION_GEAR_RATIO;
        }

        /**
         * gets current module states
         */
        public SwerveModuleState[] modState() {
                SwerveModuleState[] LFState = new SwerveModuleState[numMotors];
                for (int i = 0; i < numMotors; i++) {
                        LFState[i] = new SwerveModuleState(
                                        speedMotors[i].getSelectedSensorVelocity() * 10
                                                        / Constants.Swerve.RELATIVE_ENCODER_RATIO
                                                        * (0.05 * 2 * Math.PI),
                                        new Rotation2d(
                                                        directionMotors[i].getSelectedSensorPosition()
                                                                        / Constants.Swerve.RELATIVE_ENCODER_RATIO
                                                                        * 2 * Math.PI
                                                                        / Constants.Swerve.DIRECTION_GEAR_RATIO));
                }
                return LFState;
        }

        /**
         * run in manualswerve to get target speed
         */
        public void setTarget(ChassisSpeeds _target) {
                target = _target;
        }

        @Override
        public void periodic() {
                // converts target speeds to swerve module angle and rotations
                MOD_TARGETS = kinematics.toSwerveModuleStates(target);
                for (int i = 0; i < numMotors; i++) {
                        SmartDashboard.putNumber("ABS Encoder " + i, encoders[i].getAbsolutePosition());
                        SmartDashboard.putNumber("Offset Abs Encoder" + i, getOffsetAbs(i));
                        SmartDashboard.putNumber("M" + i, MOD_TARGETS[i].angle.getRotations());
                        SmartDashboard.putNumber("Relative " + i, getRelInRotations(i));
                        // position PIDs
                        dPID[i].setSetpoint(MOD_TARGETS[i].angle.getRotations());
                        // sets speed/position of the motors
                        speedMotors[i].set(ControlMode.PercentOutput, MOD_TARGETS[i].speedMetersPerSecond);
                        double simpleRelativeEncoderVal = ((directionMotors[i].getSelectedSensorPosition()
                                        / Constants.Swerve.RELATIVE_ENCODER_RATIO)
                                        / Constants.Swerve.DIRECTION_GEAR_RATIO);
                        directionMotors[i].set(ControlMode.PercentOutput,
                                        dPID[i].calculate(simpleRelativeEncoderVal));
                }
        }

        @Override
        public void simulationPeriodic() {
                periodic();
        }
}
