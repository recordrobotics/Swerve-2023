// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.wpilibj.SerialPort;
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
  private CANCoder[] encoders = {
      new CANCoder(RobotMap.swerve.DEVICE_NUMBER[0]),
      new CANCoder(RobotMap.swerve.DEVICE_NUMBER[1]),
      new CANCoder(RobotMap.swerve.DEVICE_NUMBER[2]),
      new CANCoder(RobotMap.swerve.DEVICE_NUMBER[3])
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

  public AHRS _nav = new AHRS(SerialPort.Port.kUSB1);
  private static final SwerveModulePosition[] startPos = {
      new SwerveModulePosition(),
      new SwerveModulePosition(),
      new SwerveModulePosition(),
      new SwerveModulePosition()
  };
  SwerveDriveOdometry m_Odometry = new SwerveDriveOdometry(kinematics, _nav.getRotation2d(), startPos);

  ChassisSpeeds target;

  public Swerve() {

    speedMotors[0].set(ControlMode.PercentOutput, 0);
    speedMotors[1].set(ControlMode.PercentOutput, 0);
    speedMotors[2].set(ControlMode.PercentOutput, 0);
    speedMotors[3].set(ControlMode.PercentOutput, 0);

    directionMotors[0].set(ControlMode.PercentOutput, 0);
    directionMotors[0].set(ControlMode.PercentOutput, 0);
    directionMotors[0].set(ControlMode.PercentOutput, 0);
    directionMotors[0].set(ControlMode.PercentOutput, 0);

  }

  // gets current module states
  public SwerveModuleState[] modState() {
    SwerveModuleState[] LFState = {
        new SwerveModuleState(speedMotors[0].getSelectedSensorVelocity() * Constants.Swerve.SPEED_GEAR_RATIO,
            new Rotation2d(
                2 * Math.PI * directionMotors[0].getSelectedSensorPosition() * Constants.Swerve.SPEED_GEAR_RATIO)),
        new SwerveModuleState(speedMotors[1].getSelectedSensorVelocity() * Constants.Swerve.SPEED_GEAR_RATIO,
            new Rotation2d(
                2 * Math.PI * directionMotors[1].getSelectedSensorPosition() * Constants.Swerve.SPEED_GEAR_RATIO)),
        new SwerveModuleState(speedMotors[2].getSelectedSensorVelocity() * Constants.Swerve.SPEED_GEAR_RATIO,
            new Rotation2d(
                2 * Math.PI * directionMotors[2].getSelectedSensorPosition() * Constants.Swerve.SPEED_GEAR_RATIO)),
        new SwerveModuleState(speedMotors[3].getSelectedSensorVelocity() * Constants.Swerve.SPEED_GEAR_RATIO,
            new Rotation2d(
                2 * Math.PI * directionMotors[3].getSelectedSensorPosition() * Constants.Swerve.SPEED_GEAR_RATIO))
    };
    return LFState;
  }

  public void setTarget(ChassisSpeeds _target) {
    target = _target;
  }

  public double getX() {
    return _nav.getDisplacementX();
  }

  public double getY() {
    return _nav.getDisplacementY();
  }

  public double getRot() {
    return Units.degreesToRadians(_nav.getYaw());
  }

  @Override
  public void periodic() {
    MOD_TARGETS = kinematics.toSwerveModuleStates(target);

    // optimises angle change
    SwerveModuleState.optimize(MOD_TARGETS[0], modState()[0].angle);
    SwerveModuleState.optimize(MOD_TARGETS[1], modState()[1].angle);
    SwerveModuleState.optimize(MOD_TARGETS[2], modState()[2].angle);
    SwerveModuleState.optimize(MOD_TARGETS[3], modState()[3].angle);

    // position PIDs
    dPID[0].setSetpoint(Constants.Swerve.DIRECTION_GEAR_RATIO * MOD_TARGETS[0].angle.getRadians());
    dPID[0].setSetpoint(Constants.Swerve.DIRECTION_GEAR_RATIO * MOD_TARGETS[1].angle.getRadians());
    dPID[2].setSetpoint(Constants.Swerve.DIRECTION_GEAR_RATIO * MOD_TARGETS[2].angle.getRadians());
    dPID[3].setSetpoint(Constants.Swerve.DIRECTION_GEAR_RATIO * MOD_TARGETS[3].angle.getRadians());

    // sets speed/position of the motors
    speedMotors[0].set(ControlMode.Velocity, MOD_TARGETS[0].speedMetersPerSecond * Constants.Swerve.SPEED_GEAR_RATIO);
    speedMotors[1].set(ControlMode.Velocity, MOD_TARGETS[1].speedMetersPerSecond * Constants.Swerve.SPEED_GEAR_RATIO);
    speedMotors[2].set(ControlMode.Velocity, MOD_TARGETS[2].speedMetersPerSecond * Constants.Swerve.SPEED_GEAR_RATIO);
    speedMotors[3].set(ControlMode.Velocity, MOD_TARGETS[3].speedMetersPerSecond * Constants.Swerve.SPEED_GEAR_RATIO);

    directionMotors[0].set(ControlMode.PercentOutput,
        dPID[0].calculate(Math.toRadians(encoders[0].getAbsolutePosition())));
    directionMotors[1].set(ControlMode.PercentOutput,
        dPID[1].calculate(Math.toRadians(encoders[1].getAbsolutePosition())));
    directionMotors[2].set(ControlMode.PercentOutput,
        dPID[2].calculate(Math.toRadians(encoders[0].getAbsolutePosition())));
    directionMotors[3].set(ControlMode.PercentOutput,
        dPID[3].calculate(Math.toRadians(encoders[0].getAbsolutePosition())));
  }

  @Override
  public void simulationPeriodic() {

  }
}
