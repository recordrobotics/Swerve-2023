// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Swerve extends SubsystemBase {

  private final double moduleWidth = 0.762;
  private final double moduleLength = 0.762;

  Translation2d m_frontLeftLocation = new Translation2d(moduleWidth / 2, moduleLength / 2);
  Translation2d m_frontRightLocation = new Translation2d(moduleWidth / 2, -(moduleLength / 2));
  Translation2d m_backLeftLocation = new Translation2d(-(moduleWidth / 2), moduleLength / 2);
  Translation2d m_backRightLocation = new Translation2d(-(moduleWidth / 2), -(moduleLength / 2));
  
  // Creating my kinematics object using the module locations
  SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);
  
  public Swerve() {}

  @Override
  public void periodic() {
    
  }

  @Override
  public void simulationPeriodic() {
    
  }
}
