// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.AimbotConstants;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.Limelight;

public class AimBotAngle extends CommandBase {
  
  Limelight limelight;

  public AimBotAngle(Limelight limelight, DrivetrainSubsystem drivetrainSubsystem) {
    drivetrainSubsystem.drive(ChassisSpeeds.fromFieldRelativeSpeeds(
        0,
        0,
        Math.toRadians(limelight.getTargetData().horizontalOffset),
        drivetrainSubsystem.getGyroscopeRotation()));

    addRequirements(limelight, drivetrainSubsystem);
    this.limelight = limelight;
  }

  @Override
  public boolean isFinished() {
    return (Math.abs(limelight.getTargetData().horizontalOffset) <= AimbotConstants.minimumAdjustment);
  }
}
