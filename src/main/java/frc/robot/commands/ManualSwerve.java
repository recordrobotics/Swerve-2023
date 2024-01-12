// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.control.IControlInput;
import frc.robot.subsystems.Swerve;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class ManualSwerve extends CommandBase {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private Swerve _swerve;
  private IControlInput _controls;

  public ChassisSpeeds target;
  private static final double SPEED = 1;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ManualSwerve(Swerve swerve, IControlInput controls) {
    _swerve = swerve;
    _controls = controls;
    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    /**
     * Target Velocity and Angle
     */
    _swerve.setTarget(ChassisSpeeds.fromFieldRelativeSpeeds(
        _controls.getX() * SPEED, _controls.getY() * SPEED, _controls.setSpin(), _swerve.getAngle()));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
