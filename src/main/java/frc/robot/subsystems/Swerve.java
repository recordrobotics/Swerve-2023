// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Swerve extends SubsystemBase {

  public static final double GEAR_RATIO = 1;

  // TODO change to correct motor
  private TalonFX[] speedMotors = {
      new TalonFX(RobotMap.swerve.LFS_MOTOR_PORT),
      new TalonFX(RobotMap.swerve.LBS_MOTOR_PORT),
      new TalonFX(RobotMap.swerve.RFS_MOTOR_PORT),
      new TalonFX(RobotMap.swerve.RBS_MOTOR_PORT)
  };
  private TalonFX[] directionMotors = {
      new TalonFX(RobotMap.swerve.LFD_MOTOR_PORT),
      new TalonFX(RobotMap.swerve.LBD_MOTOR_PORT),
      new TalonFX(RobotMap.swerve.RFD_MOTOR_PORT),
      new TalonFX(RobotMap.swerve.RBD_MOTOR_PORT)
  };
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
  private static final SwerveModulePosition[] startPos = {
      new SwerveModulePosition(),
      new SwerveModulePosition(),
      new SwerveModulePosition(),
      new SwerveModulePosition()
  };
  SwerveDriveOdometry m_Odometry = new SwerveDriveOdometry(kinematics, _nav.getRotation2d(), startPos);

  ChassisSpeeds target;

  public Swerve() {
    speedMotors[0].set(ControlMode.Velocity, 0);
    speedMotors[1].set(ControlMode.Velocity, 0);
    speedMotors[2].set(ControlMode.Velocity, 0);
    speedMotors[3].set(ControlMode.Velocity, 0);

    directionMotors[0].set(ControlMode.Position, 0);
    directionMotors[0].set(ControlMode.Position, 0);
    directionMotors[0].set(ControlMode.Position, 0);
    directionMotors[0].set(ControlMode.Position, 0);

  }
  //gets states of modules
  public SwerveModuleState[] modState() {
    SwerveModuleState[] LFState = {
        new SwerveModuleState(speedMotors[0].getSelectedSensorVelocity() * GEAR_RATIO,
            new Rotation2d(2 * Math.PI * directionMotors[0].getSelectedSensorPosition() * GEAR_RATIO)),
        new SwerveModuleState(speedMotors[1].getSelectedSensorVelocity() * GEAR_RATIO,
            new Rotation2d(2 * Math.PI * directionMotors[1].getSelectedSensorPosition() * GEAR_RATIO)),
        new SwerveModuleState(speedMotors[2].getSelectedSensorVelocity() * GEAR_RATIO,
            new Rotation2d(2 * Math.PI * directionMotors[2].getSelectedSensorPosition() * GEAR_RATIO)),
        new SwerveModuleState(speedMotors[3].getSelectedSensorVelocity() * GEAR_RATIO,
            new Rotation2d(2 * Math.PI * directionMotors[3].getSelectedSensorPosition() * GEAR_RATIO))
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
    //optimises angles
    SwerveModuleState.optimize(MOD_TARGETS[0], modState()[0].angle);
    SwerveModuleState.optimize(MOD_TARGETS[1], modState()[1].angle);
    SwerveModuleState.optimize(MOD_TARGETS[2], modState()[2].angle);
    SwerveModuleState.optimize(MOD_TARGETS[3], modState()[3].angle);
    //sets speed
    speedMotors[0].set(ControlMode.Velocity, MOD_TARGETS[0].speedMetersPerSecond);
    speedMotors[1].set(ControlMode.Velocity, MOD_TARGETS[1].speedMetersPerSecond);
    speedMotors[2].set(ControlMode.Velocity, MOD_TARGETS[2].speedMetersPerSecond);
    speedMotors[3].set(ControlMode.Velocity, MOD_TARGETS[3].speedMetersPerSecond);
    //sets angles
    directionMotors[0].set(ControlMode.Position, MOD_TARGETS[0].angle.getDegrees() * 256 / 45);
    directionMotors[0].set(ControlMode.Position, MOD_TARGETS[1].angle.getDegrees() * 256 / 45);
    directionMotors[0].set(ControlMode.Position, MOD_TARGETS[2].angle.getDegrees() * 256 / 45);
    directionMotors[0].set(ControlMode.Position, MOD_TARGETS[3].angle.getDegrees() * 256 / 45);
  }

  @Override
  public void simulationPeriodic() {

  }
}
