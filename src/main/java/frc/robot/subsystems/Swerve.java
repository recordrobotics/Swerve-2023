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

    private SwerveModuleState[] MOD_TARGETS;

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

    ChassisSpeeds target = new ChassisSpeeds();

    public Swerve() {
        
        encoders[0].setPositionOffset(0.089);
        encoders[1].setPositionOffset(0.416);
        encoders[2].setPositionOffset(0.865);
        encoders[3].setPositionOffset(0.195);
        
        speedMotors[0].set(ControlMode.PercentOutput, 0);
        speedMotors[1].set(ControlMode.PercentOutput, 0);
        speedMotors[2].set(ControlMode.PercentOutput, 0);
        speedMotors[3].set(ControlMode.PercentOutput, 0);

        directionMotors[0].set(ControlMode.PercentOutput, 0);
        directionMotors[1].set(ControlMode.PercentOutput, 0);
        directionMotors[2].set(ControlMode.PercentOutput, 0);
        directionMotors[3].set(ControlMode.PercentOutput, 0);

        directionMotors[0].setSelectedSensorPosition(encoders[0].getAbsolutePosition() * 2048);
        directionMotors[1].setSelectedSensorPosition(encoders[1].getAbsolutePosition() * 2048);
        directionMotors[2].setSelectedSensorPosition(encoders[2].getAbsolutePosition() * 2048);
        directionMotors[3].setSelectedSensorPosition(encoders[3].getAbsolutePosition() * 2048);
    }

    // gets current module states
    public SwerveModuleState[] modState() {
        SwerveModuleState[] LFState = {
                new SwerveModuleState(speedMotors[0].getSelectedSensorVelocity() * 10/2048 * (0.05 * 2 * Math.PI),
                        new Rotation2d(
                                directionMotors[0].getSelectedSensorPosition() / 2048 * 2 * Math.PI)),
                new SwerveModuleState(speedMotors[1].getSelectedSensorVelocity() * 10/2048 * (0.05 * 2 * Math.PI),
                        new Rotation2d(
                                directionMotors[1].getSelectedSensorPosition() / 2048 * 2 * Math.PI)),
                new SwerveModuleState(speedMotors[2].getSelectedSensorVelocity() * 10/2048 * (0.05 * 2 * Math.PI),
                        new Rotation2d(
                                directionMotors[2].getSelectedSensorPosition() / 2048 * 2 * Math.PI)),
                new SwerveModuleState(speedMotors[3].getSelectedSensorVelocity() * 10/2048 * (0.05 * 2 * Math.PI),
                        new Rotation2d(
                                directionMotors[3].getSelectedSensorPosition() / 2048 * 2 * Math.PI))
        };
        return LFState;
    }

    public void setTarget(ChassisSpeeds _target) {
        target = _target;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("0", encoders[0].getAbsolutePosition());
        SmartDashboard.putNumber("1", encoders[1].getAbsolutePosition());
        SmartDashboard.putNumber("2", encoders[2].getAbsolutePosition());
        SmartDashboard.putNumber("3", encoders[3].getAbsolutePosition());

        MOD_TARGETS = kinematics.toSwerveModuleStates(target);

        // optimises angle change
        SwerveModuleState.optimize(MOD_TARGETS[0], modState()[0].angle);
        SwerveModuleState.optimize(MOD_TARGETS[1], modState()[1].angle);
        SwerveModuleState.optimize(MOD_TARGETS[2], modState()[2].angle);
        SwerveModuleState.optimize(MOD_TARGETS[3], modState()[3].angle);

        // position PIDs
        dPID[0].setSetpoint(MOD_TARGETS[0].angle.getRotations());
        dPID[0].setSetpoint(MOD_TARGETS[1].angle.getRotations());
        dPID[2].setSetpoint(MOD_TARGETS[2].angle.getRotations());
        dPID[3].setSetpoint(MOD_TARGETS[3].angle.getRotations());

        // sets speed/position of the motors
        speedMotors[0].set(ControlMode.PercentOutput, MOD_TARGETS[0].speedMetersPerSecond);
        speedMotors[1].set(ControlMode.PercentOutput, MOD_TARGETS[1].speedMetersPerSecond);
        speedMotors[2].set(ControlMode.PercentOutput, MOD_TARGETS[2].speedMetersPerSecond);
        speedMotors[3].set(ControlMode.PercentOutput, MOD_TARGETS[3].speedMetersPerSecond);

        directionMotors[0].set(ControlMode.PercentOutput,
                dPID[0].calculate(directionMotors[0].getSelectedSensorPosition() / 2048));
        directionMotors[1].set(ControlMode.PercentOutput,
                dPID[1].calculate(directionMotors[1].getSelectedSensorPosition() / 2048));
        directionMotors[2].set(ControlMode.PercentOutput,
                dPID[2].calculate(directionMotors[2].getSelectedSensorPosition() / 2048));
        directionMotors[3].set(ControlMode.PercentOutput,
                dPID[3].calculate(directionMotors[3].getSelectedSensorPosition() / 2048));
    }

    @Override
    public void simulationPeriodic() {

    }
}
