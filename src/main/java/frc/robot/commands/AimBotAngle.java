// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.AimbotConstants;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.Photon;

public class AimBotAngle extends CommandBase {

  Photon photon;
  DrivetrainSubsystem drivetrainSubsystem;

  public AimBotAngle(Photon limelight, DrivetrainSubsystem drivetrainSubsystem) {
    addRequirements(limelight, drivetrainSubsystem);
    this.photon = limelight;
    this.drivetrainSubsystem = drivetrainSubsystem;
  }

  @Override
  public void execute() {
    drivetrainSubsystem.drive(new ChassisSpeeds(
        0,
        0,
        Math.toRadians(photon.xOffset) * 7.2)
        );
  }

  @Override
  public void end(boolean interrupted) {
    drivetrainSubsystem.drive(new ChassisSpeeds(0, 0, 0));
  }

  @Override
  public boolean isFinished() {
    return (Math.abs(photon.xOffset) <= AimbotConstants.minimumAdjustment);
  }
}
