// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.ConveyorConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.ColorSensor;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LidarSubsystem;
import frc.robot.subsystems.Limelight;

public class Capture extends InstantCommand {

  Limelight limelight;
  LidarSubsystem lidar;
  DrivetrainSubsystem drivetrainSubsystem;
  IntakeSubsystem intakeSubsystem;
  ColorSensor colorSensor;

  public int alliance = 1; // Red (temp)
  double stoppingDistance = 1; //Temp Stopping Distance

  public Capture(Limelight limelight, LidarSubsystem lidarSubsystem, DrivetrainSubsystem drivetrainSubsystem, 
  IntakeSubsystem intakeSubsystem, ColorSensor colorSensor) {
    addRequirements(limelight, lidarSubsystem, drivetrainSubsystem, intakeSubsystem);
    lidar = lidarSubsystem;
    this.limelight = limelight;
    this.drivetrainSubsystem = drivetrainSubsystem;
    this.intakeSubsystem = intakeSubsystem;
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
    drivetrainSubsystem.drive(ChassisSpeeds.fromFieldRelativeSpeeds(stoppingDistance, stoppingDistance, 3.14159, currentRotation));
    intakeSubsystem.intakeBall(IntakeConstants.INTAKE_SPEED);

    //Using PathPlanner we would:
    // Generate Trajectory to Ball
    // Add a point in the middle of the trajectory to rotate 180 and Activate intake
    // Once reaching the point, we stop our intake(inside the end | when it hits the color sensor(also a end condition))
  }

  @Override
  public void end(boolean interrupted) {
    limelight.setPipeline(0);
    intakeSubsystem.stopIntake();
  }
}
