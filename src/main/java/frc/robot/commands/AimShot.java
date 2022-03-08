// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.ShooterSubsystem;

public class AimShot extends CommandBase {
  ShooterSubsystem shooterSubsystem;
  Limelight limelight;

  public AimShot(ShooterSubsystem shooterSubsystem, Limelight limelight) {
    this.shooterSubsystem = shooterSubsystem;
    this.limelight = limelight;

    addRequirements(shooterSubsystem, limelight);
  }

  @Override
  public void execute() {
    double m = 0.000617973;
    double b = 0.433686;
    double distance = limelight.getDistance();

    double speed = m * distance + b;
    shooterSubsystem.rampUpShooter(speed);
  }
  
  @Override
  public void end(boolean interrupted){
    shooterSubsystem.rampDownShooter();
  }
}
