// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ShooterSubsystem;

public class AimShotPID extends PIDCommand {
  ShooterSubsystem shooterSubsystem;

  Timer timer = new Timer();
  boolean started = false;

  public AimShotPID(ShooterSubsystem shooterSubsystem, double setpoint) {
    super(
        // The controller that the command will use
        new PIDController(ShooterConstants.kP, 0, 0),
        // This should return the measurement
        () -> shooterSubsystem.getVelocity(),
        // This should return the setpoint (can also be a constant)
        () -> setpoint,
        // This uses the output
        output -> {
          // Use the output here
          double feedforward = shooterSubsystem.calculateFeedforward(setpoint);
          shooterSubsystem.setVoltage(output + feedforward);
        });

    getController().setTolerance(100);
    addRequirements(shooterSubsystem);
    this.shooterSubsystem = shooterSubsystem;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // if (getController().atSetpoint()&& !started){
    //   timer.start();
    //   started = true;
    // } 
    // return getController().atSetpoint() && timer.hasElapsed(3);
    return getController().atSetpoint();
  }

  // @Override
  // public void end(boolean interrupted) {
  //   shooterSubsystem.rampDownShooter();
  // }
}
