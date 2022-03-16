// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.LidarSubsystem;
import frc.robot.subsystems.Limelight;

public class SearchAndCapture extends CommandBase {

  Limelight limelight;
  LidarSubsystem lidar;
  DrivetrainSubsystem drivetrainSubsystem;

  // Temp for now,replace with a getAlliance();
  public int alliance = 1;

  double stoppingDistance = 1;

  public SearchAndCapture(Limelight limelight, LidarSubsystem lidarSubsystem, DrivetrainSubsystem drivetrainSubsystem) {
    addRequirements(limelight, drivetrainSubsystem);
    lidar = lidarSubsystem;
    this.limelight = limelight;
    this.drivetrainSubsystem = drivetrainSubsystem;
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    double targetAngle = limelight.getTargetData().horizontalOffset;
    limelight.setPipeline(alliance);

    // Rotate at a constant rate to find the ball
    if (targetAngle == 0)
      drivetrainSubsystem.drive(new ChassisSpeeds(0, 0, 0.25));

    // rotate toward that angle
    drivetrainSubsystem.drive(ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, Math.toRadians(targetAngle),
        drivetrainSubsystem.getGyroscopeRotation()));

    Rotation2d currentRotation = drivetrainSubsystem.getGyroscopeRotation();
    double distance = lidar.getDistance(false);
    // double distance = limelight.getDistance();

    double x = distance * currentRotation.getCos();
    double y = distance * currentRotation.getSin();

    drivetrainSubsystem.drive(
      ChassisSpeeds.fromFieldRelativeSpeeds(x - stoppingDistance, y - stoppingDistance, 0, currentRotation));

    // Generate a trajectory and move towards that but for now
    // With Path Planner, use the overriding angle to force the angle to be currentRot + 180 to simplify and to ensure accuracy
    drivetrainSubsystem.drive(ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, 3.14159, currentRotation));
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
