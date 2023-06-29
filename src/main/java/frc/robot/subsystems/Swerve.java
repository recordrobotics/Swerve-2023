// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Swerve extends SubsystemBase {

  // TODO change to correct motor
  private TalonFX LFS_MOTOR = new TalonFX(RobotMap.swerve.LFS_MOTOR_PORT);
  private TalonFX LBS_MOTOR = new TalonFX(RobotMap.swerve.LBS_MOTOR_PORT);
  private TalonFX RFS_MOTOR = new TalonFX(RobotMap.swerve.RFS_MOTOR_PORT);
  private TalonFX RBS_MOTOR = new TalonFX(RobotMap.swerve.RBS_MOTOR_PORT);

  private TalonFX LFD_MOTOR = new TalonFX(RobotMap.swerve.LFD_MOTOR_PORT);
  private TalonFX LBD_MOTOR = new TalonFX(RobotMap.swerve.LBD_MOTOR_PORT);
  private TalonFX RFD_MOTOR = new TalonFX(RobotMap.swerve.RFD_MOTOR_PORT);
  private TalonFX RBD_MOTOR = new TalonFX(RobotMap.swerve.RBD_MOTOR_PORT);

  private final double moduleWidth = 0.762;
  private final double moduleLength = 0.762;

  Translation2d m_frontLeftLocation = new Translation2d(moduleWidth / 2, moduleLength / 2);
  Translation2d m_frontRightLocation = new Translation2d(moduleWidth / 2, -(moduleLength / 2));
  Translation2d m_backLeftLocation = new Translation2d(-(moduleWidth / 2), moduleLength / 2);
  Translation2d m_backRightLocation = new Translation2d(-(moduleWidth / 2), -(moduleLength / 2));

  SwerveDriveKinematics kinematics = new SwerveDriveKinematics(m_frontLeftLocation, m_frontRightLocation,
      m_backLeftLocation, m_backRightLocation);

  private SwerveModuleState[] MOD_TARGETS;

  AHRS _nav = new AHRS(SerialPort.Port.kUSB1);
  SwerveDriveOdometry m_Odometry = new SwerveDriveOdometry(kinematics, _nav.getRotation2d());

  ChassisSpeeds target;

  private static final TalonFXConfiguration swerveConfig = new TalonFXConfiguration();

  public Swerve() {
    LFS_MOTOR.configAllSettings(swerveConfig);
    LBS_MOTOR.configAllSettings(swerveConfig);
    RFS_MOTOR.configAllSettings(swerveConfig);
    RBS_MOTOR.configAllSettings(swerveConfig);

    LFD_MOTOR.configAllSettings(swerveConfig);
    LBD_MOTOR.configAllSettings(swerveConfig);
    RFD_MOTOR.configAllSettings(swerveConfig);
    RBD_MOTOR.configAllSettings(swerveConfig);

    LFS_MOTOR.set(ControlMode.Velocity, 0);
    LBS_MOTOR.set(ControlMode.Velocity, 0);
    RFS_MOTOR.set(ControlMode.Velocity, 0);
    RBS_MOTOR.set(ControlMode.Velocity, 0);

    LFD_MOTOR.set(ControlMode.Position, 0);
    LBD_MOTOR.set(ControlMode.Position, 0);
    RFD_MOTOR.set(ControlMode.Position, 0);
    RBD_MOTOR.set(ControlMode.Position, 0);

  }

  public SwerveModuleState LFState() {
    return new SwerveModuleState(LFS_MOTOR.getSelectedSensorVelocity(),
        new Rotation2d(2 * Math.PI * LFD_MOTOR.getSelectedSensorPosition()));
  }

  public SwerveModuleState LBState() {
    return new SwerveModuleState(LBS_MOTOR.getSelectedSensorVelocity(),
        new Rotation2d(2 * Math.PI * LBD_MOTOR.getSelectedSensorPosition()));
  }

  public SwerveModuleState RFState() {
    return new SwerveModuleState(RFS_MOTOR.getSelectedSensorVelocity(),
        new Rotation2d(2 * Math.PI * RFD_MOTOR.getSelectedSensorPosition()));
  }

  public SwerveModuleState RBState() {
    return new SwerveModuleState(RBS_MOTOR.getSelectedSensorVelocity(),
        new Rotation2d(2 * Math.PI * RBD_MOTOR.getSelectedSensorPosition()));
  }

  public void setTarget(ChassisSpeeds _target) {
    target = _target;
  }

  @Override
  public void periodic() {
    MOD_TARGETS = kinematics.toSwerveModuleStates(target);

    SwerveModuleState.optimize(MOD_TARGETS[0], LFState().angle);
    SwerveModuleState.optimize(MOD_TARGETS[1], LFState().angle);
    SwerveModuleState.optimize(MOD_TARGETS[2], LFState().angle);
    SwerveModuleState.optimize(MOD_TARGETS[3], LFState().angle);

    LFS_MOTOR.set(ControlMode.Velocity, MOD_TARGETS[0].speedMetersPerSecond);
    LBS_MOTOR.set(ControlMode.Velocity, MOD_TARGETS[1].speedMetersPerSecond);
    RFS_MOTOR.set(ControlMode.Velocity, MOD_TARGETS[2].speedMetersPerSecond);
    RBS_MOTOR.set(ControlMode.Velocity, MOD_TARGETS[3].speedMetersPerSecond);

    LFD_MOTOR.set(ControlMode.Position, MOD_TARGETS[0].angle.getDegrees() * 256 / 45);
    LBD_MOTOR.set(ControlMode.Position, MOD_TARGETS[1].angle.getDegrees() * 256 / 45);
    RFD_MOTOR.set(ControlMode.Position, MOD_TARGETS[2].angle.getDegrees() * 256 / 45);
    RBD_MOTOR.set(ControlMode.Position, MOD_TARGETS[3].angle.getDegrees() * 256 / 45);
  }

  @Override
  public void simulationPeriodic() {

  }
}
