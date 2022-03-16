// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.LidarSubsystem;
import frc.robot.subsystems.Limelight;

public class Capture extends InstantCommand {

  Limelight limelight;
  LidarSubsystem lidar;
  DrivetrainSubsystem drivetrainSubsystem;

  public int alliance = 1; // Red (temp)
  double stoppingDistance = 1; //Temp Stopping Distance

  public Capture(Limelight limelight, LidarSubsystem lidarSubsystem, DrivetrainSubsystem drivetrainSubsystem) {
    addRequirements(limelight, lidarSubsystem, drivetrainSubsystem);
    lidar = lidarSubsystem;
    this.limelight = limelight;
    this.drivetrainSubsystem = drivetrainSubsystem;
  }

  @Override
  public void execute() {
    Rotation2d currentRotation = drivetrainSubsystem.getGyroscopeRotation();
    double distance = lidar.getDistance(false);

    Translation2d pos = new Translation2d(distance * currentRotation.getCos(), distance * currentRotation.getSin());

    drivetrainSubsystem.drive(
      ChassisSpeeds.fromFieldRelativeSpeeds(pos.getX() - stoppingDistance, pos.getX() - stoppingDistance, 0, currentRotation));

    // Generate a trajectory and move towards
    // With Path Planner, use the overriding angle to force the angle to be currentRot + 180 to simplify and to ensure accuracy
    // Current Temp implementation
    drivetrainSubsystem.drive(ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, 3.14159, currentRotation));

    drivetrainSubsystem.drive(new ChassisSpeeds(stoppingDistance, stoppingDistance, 0));

    //Move Conveyor and capture ball
  }

  @Override
  public void end(boolean interrupted) {
    limelight.setPipeline(0);
  }
}
