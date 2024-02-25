// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.SwerveSubsystem;

public class RobotContainer {
  SwerveSubsystem drivebase = new SwerveSubsystem();
  CommandXboxController driverXbox = new CommandXboxController(0);

  public RobotContainer() {
    Command driveFieldOrientedDirectAngle = drivebase.driveCommand(
        () -> -MathUtil.applyDeadband(driverXbox.getLeftY(), 0.1),
        () -> -MathUtil.applyDeadband(driverXbox.getLeftX(), 0.1),
        () -> -driverXbox.getRightX(),
        () -> -driverXbox.getRightY());

    Command driveFieldOrientedAnglularVelocity = drivebase.driveCommand(
      () -> -MathUtil.applyDeadband(driverXbox.getLeftY(), 0.1),
      () -> -MathUtil.applyDeadband(driverXbox.getLeftX(), 0.1),
      () -> -driverXbox.getRightX());

    drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);

    configureBindings();
  }

  private void configureBindings() {
    driverXbox.start().whileTrue(drivebase.zeroGyro());
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
