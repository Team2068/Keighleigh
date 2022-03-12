// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.sql.Driver;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.ShooterSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AimShotCalculated extends PIDCommand {
  /** Creates a new AimShotPID. */

  ShooterSubsystem shooterSubsystem;

  public AimShotCalculated(ShooterSubsystem shooterSubsystem, Limelight limelight) {
    super(
        // The controller that the command will use
        new PIDController(ShooterConstants.kP, 0, 0),
        // This should return the measurement
        () -> shooterSubsystem.getVelocity(),
        // This should return the setpoint (can also be a constant)
        () -> {
          double rpm = limelight.distanceToRpm();
          DriverStation.reportWarning("Calculated RPM: " + rpm, false);
          SmartDashboard.putNumber("Calculated RPM", rpm);
          return rpm;
        },
        // This uses the output
        output -> {
          // Use the output here

          // hacky way to keep state between parameters, hopefully the distance doesn't change!
          double rpm = limelight.distanceToRpm();

          double feedforward = shooterSubsystem.calculateFeedforward(rpm);
          shooterSubsystem.setVoltage(output + feedforward);
        });
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
    addRequirements(shooterSubsystem);
    this.shooterSubsystem = shooterSubsystem;
    getController().setTolerance(100);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return getController().atSetpoint();
  }
}
