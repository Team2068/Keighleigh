// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.ShooterSubsystem;

public class AimShot extends CommandBase {
  ShooterSubsystem shooterSubsystem;
  Limelight limelight;

  Timer time = new Timer();

  public AimShot(ShooterSubsystem shooterSubsystem, Limelight limelight) {
    this.shooterSubsystem = shooterSubsystem;
    this.limelight = limelight;
    addRequirements(shooterSubsystem, limelight);
  }

  @Override
  public void initialize() {
    time.start();
  }

  @Override
  public void execute() {
    double m = 0.000617973;
    double b = 0.433686 + 0.05;
    double distance = limelight.getDistance();

    double speed = (m * distance) + b;
    shooterSubsystem.rampUpShooter(speed);
    DriverStation.reportWarning("D: " + distance + " P:" + speed, false);
  }
  
  @Override
  public void end(boolean interrupted){
    shooterSubsystem.rampDownShooter();
  }

  // @Override
  // public boolean isFinished() {
  //   return time.get() >= 3; // done after 3 seconds
  // }
}
