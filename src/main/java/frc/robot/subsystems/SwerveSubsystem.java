// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.io.File;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import swervelib.parser.PIDFConfig;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;
import swervelib.SwerveDrive;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;

public class SwerveSubsystem extends SubsystemBase {
  SwerveDrive swerveDrive;
  // double maximumSpeed = Units.feetToMeters(16.6);
  double maximumSpeed = Units.feetToMeters(16.6);
  double headingHoldAngle = 0;

  /** Creates a new SwerveSubsystem. */
  public SwerveSubsystem() {
    SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
    try {
      File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(), "neo/swerve");
      swerveDrive = new SwerveParser(swerveJsonDirectory).createSwerveDrive(maximumSpeed);
    } catch (Exception e) {
      throw new RuntimeException(e);
    }

     swerveDrive.setHeadingCorrection(true);

     SmartDashboard.putNumber("Heading", getHeading().getDegrees());
  }

  public void drive(ChassisSpeeds velocity) {
    swerveDrive.drive(velocity);
  }

  public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier headingX,
      DoubleSupplier headingY) {
    // swerveDrive.setHeadingCorrection(true);
    return run(() -> {
      double xInput = Math.pow(translationX.getAsDouble(), 3); // Smooth controllout
      double yInput = Math.pow(translationY.getAsDouble(), 3); // Smooth controllout
      // Make the robot move
      swerveDrive.driveFieldOriented(swerveDrive.swerveController.getTargetSpeeds(xInput,
          yInput,
          headingX.getAsDouble(),
          headingY.getAsDouble(),
          // swerveDrive.getYaw().getRadians(),
          swerveDrive.getOdometryHeading().getRadians(),
          swerveDrive.getMaximumVelocity()));
    });


  //  SmartDashboard.put ("sdf", swerveDrive.swerveController.config.headingPIDF.createPIDController());
  }






  public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY,
      DoubleSupplier angularRotationX) {
   
    return run(() -> {
      double calculatedAngularRotation = 0;

      if(Math.abs(angularRotationX.getAsDouble()) > 0.1) {
        this.headingHoldAngle = swerveDrive.getOdometryHeading().getRadians();
        // Slow things down.
        calculatedAngularRotation = angularRotationX.getAsDouble() * swerveDrive.getMaximumAngularVelocity() * .75;
      } else {
        calculatedAngularRotation = swerveDrive.swerveController.headingCalculate(
          swerveDrive.getOdometryHeading().getRadians(),
          headingHoldAngle
        );
      }

      // Make the robot move
      swerveDrive.drive(new Translation2d(Math.pow(translationX.getAsDouble(), 3) * swerveDrive.getMaximumVelocity(),
          Math.pow(translationY.getAsDouble(), 3) * swerveDrive.getMaximumVelocity()),
          calculatedAngularRotation,
          true,
          false);
    });
  }










  public Command setHeadingHoldAngle(double angleInDegree) {
    return runOnce(() -> {
      this.headingHoldAngle = Units.degreesToRadians(angleInDegree);
    });
  }





  public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, Rotation2d angle) {
    xInput = Math.pow(xInput, 3);
    yInput = Math.pow(yInput, 3);
    return swerveDrive.swerveController.getTargetSpeeds(xInput,
        yInput,
        angle.getRadians(),
        getHeading().getRadians(),
        maximumSpeed);
  }

  public Rotation2d getHeading() {
    return getPose().getRotation();
  }

  public Pose2d getPose() {
    return swerveDrive.getPose();
  }

  public Command zeroGyro() {
    return runOnce(() -> {
      this.headingHoldAngle = 0;
      this.swerveDrive.zeroGyro();
    });
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Heading", getHeading().getDegrees());
    // This method will be called once per scheduler run
  }
}
