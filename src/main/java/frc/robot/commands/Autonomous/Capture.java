// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.ColorSensor;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LidarSubsystem;
import frc.robot.subsystems.Limelight;

public class Capture extends CommandBase {

  Limelight limelight;
  LidarSubsystem lidar;
  DrivetrainSubsystem drivetrainSubsystem;
  IntakeSubsystem intakeSubsystem;
  ColorSensor colorSensor;

  Timer timer = new Timer();

  public Capture(Limelight limelight, LidarSubsystem lidarSubsystem, DrivetrainSubsystem drivetrainSubsystem,
      IntakeSubsystem intakeSubsystem, ColorSensor colorSensor) {
    //Adjust TimeOut
    withTimeout(5);
    addRequirements(limelight, lidarSubsystem, drivetrainSubsystem, intakeSubsystem, colorSensor);
    lidar = lidarSubsystem;
    this.limelight = limelight;
    this.drivetrainSubsystem = drivetrainSubsystem;
    this.intakeSubsystem = intakeSubsystem;
    this.colorSensor = colorSensor;
  }

  @Override
  public void execute() {
    Rotation2d currentRotation = drivetrainSubsystem.getGyroscopeRotation();
    double distance = lidar.getDistance(false);

    Translation2d pos = new Translation2d(distance * currentRotation.getCos(), distance * currentRotation.getSin());

    timer.start();

    drivetrainSubsystem.drive(
        ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, Math.PI, currentRotation));

    until(() -> timer.hasElapsed(1));

    drivetrainSubsystem.drive(
        ChassisSpeeds.fromFieldRelativeSpeeds(pos.getX(), pos.getY(), 0,
            drivetrainSubsystem.getGyroscopeRotation()));
    intakeSubsystem.intakeBall(IntakeConstants.INTAKE_SPEED);
  }

  @Override
  public boolean isFinished() {
    return colorSensor.occupiedLower();
  }

  @Override
  public void end(boolean interrupted) {
    drivetrainSubsystem.drive(new ChassisSpeeds(0,0,0));
    intakeSubsystem.stopIntake();
  }
}
