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

  public static final double GEAR_RATIO = 1;

  // TODO change to correct motor
  private TalonFX LFSmotor = new TalonFX(RobotMap.swerve.LFS_MOTOR_PORT);
  private TalonFX LBSmotor = new TalonFX(RobotMap.swerve.LBS_MOTOR_PORT);
  private TalonFX RFSmotor = new TalonFX(RobotMap.swerve.RFS_MOTOR_PORT);
  private TalonFX RBSmotor = new TalonFX(RobotMap.swerve.RBS_MOTOR_PORT);

  private TalonFX LFDmotor = new TalonFX(RobotMap.swerve.LFD_MOTOR_PORT);
  private TalonFX LBDmotor = new TalonFX(RobotMap.swerve.LBD_MOTOR_PORT);
  private TalonFX RFDmotor = new TalonFX(RobotMap.swerve.RFD_MOTOR_PORT);
  private TalonFX RBDmotor = new TalonFX(RobotMap.swerve.RBD_MOTOR_PORT);

  private final double moduleWidth = 0.762;
  private final double moduleLength = 0.762;

  Translation2d m_frontLeftLocation = new Translation2d(moduleWidth / 2, moduleLength / 2);
  Translation2d m_frontRightLocation = new Translation2d(moduleWidth / 2, -(moduleLength / 2));
  Translation2d m_backLeftLocation = new Translation2d(-(moduleWidth / 2), moduleLength / 2);
  Translation2d m_backRightLocation = new Translation2d(-(moduleWidth / 2), -(moduleLength / 2));

  SwerveDriveKinematics kinematics = new SwerveDriveKinematics(m_frontLeftLocation, m_frontRightLocation,
      m_backLeftLocation, m_backRightLocation);

  private SwerveModuleState[] MOD_TARGETS;

  public AHRS _nav = new AHRS(SerialPort.Port.kUSB1);
  SwerveDriveOdometry m_Odometry = new SwerveDriveOdometry(kinematics, _nav.getRotation2d());

  ChassisSpeeds target;

  private static final TalonFXConfiguration swerveConfig = new TalonFXConfiguration();

  public Swerve() {
    LFSmotor.configAllSettings(swerveConfig);
    LBSmotor.configAllSettings(swerveConfig);
    RFSmotor.configAllSettings(swerveConfig);
    RBSmotor.configAllSettings(swerveConfig);

    LFDmotor.configAllSettings(swerveConfig);
    LBDmotor.configAllSettings(swerveConfig);
    RFDmotor.configAllSettings(swerveConfig);
    RBDmotor.configAllSettings(swerveConfig);

    LFSmotor.set(ControlMode.Velocity, 0);
    LBSmotor.set(ControlMode.Velocity, 0);
    RFSmotor.set(ControlMode.Velocity, 0);
    RBSmotor.set(ControlMode.Velocity, 0);

    LFDmotor.set(ControlMode.Position, 0);
    LBDmotor.set(ControlMode.Position, 0);
    RFDmotor.set(ControlMode.Position, 0);
    RBDmotor.set(ControlMode.Position, 0);

  }

  public SwerveModuleState LFState() {
    return new SwerveModuleState(LFSmotor.getSelectedSensorVelocity() * GEAR_RATIO,
        new Rotation2d(2 * Math.PI * LFDmotor.getSelectedSensorPosition() * GEAR_RATIO));
  }

  public SwerveModuleState LBState() {
    return new SwerveModuleState(LBSmotor.getSelectedSensorVelocity() * GEAR_RATIO,
        new Rotation2d(2 * Math.PI * LBDmotor.getSelectedSensorPosition() * GEAR_RATIO));
  }

  public SwerveModuleState RFState() {
    return new SwerveModuleState(RFSmotor.getSelectedSensorVelocity() * GEAR_RATIO,
        new Rotation2d(2 * Math.PI * RFDmotor.getSelectedSensorPosition() * GEAR_RATIO));
  }

  public SwerveModuleState RBState() {
    return new SwerveModuleState(RBSmotor.getSelectedSensorVelocity(),
        new Rotation2d(2 * Math.PI * RBDmotor.getSelectedSensorPosition() * GEAR_RATIO));
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
    return _nav.getDisplacementX();
  }

  @Override
  public void periodic() {
    MOD_TARGETS = kinematics.toSwerveModuleStates(target);

    SwerveModuleState.optimize(MOD_TARGETS[0], LFState().angle);
    SwerveModuleState.optimize(MOD_TARGETS[1], LFState().angle);
    SwerveModuleState.optimize(MOD_TARGETS[2], LFState().angle);
    SwerveModuleState.optimize(MOD_TARGETS[3], LFState().angle);

    LFSmotor.set(ControlMode.Velocity, MOD_TARGETS[0].speedMetersPerSecond);
    LBSmotor.set(ControlMode.Velocity, MOD_TARGETS[1].speedMetersPerSecond);
    RFSmotor.set(ControlMode.Velocity, MOD_TARGETS[2].speedMetersPerSecond);
    RBSmotor.set(ControlMode.Velocity, MOD_TARGETS[3].speedMetersPerSecond);

    LFDmotor.set(ControlMode.Position, MOD_TARGETS[0].angle.getDegrees() * 256 / 45);
    LBDmotor.set(ControlMode.Position, MOD_TARGETS[1].angle.getDegrees() * 256 / 45);
    RFDmotor.set(ControlMode.Position, MOD_TARGETS[2].angle.getDegrees() * 256 / 45);
    RBDmotor.set(ControlMode.Position, MOD_TARGETS[3].angle.getDegrees() * 256 / 45);
  }

  @Override
  public void simulationPeriodic() {

  }
}
