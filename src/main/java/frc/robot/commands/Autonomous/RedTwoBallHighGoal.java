// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AimAndFire;
// import frc.robot.subsystems.ColorSensor;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.ShooterSubsystem;

public class RedTwoBallHighGoal extends SequentialCommandGroup {
  public RedTwoBallHighGoal(IntakeSubsystem intakeSubsystem, DrivetrainSubsystem drivetrainSubsystem, ShooterSubsystem shooterSubsystem, Limelight limelight, ConveyorSubsystem conveyorSubsystem) {
    addCommands(
      new InstantCommand(intakeSubsystem::controlIntakeSolenoids),
      new ParallelDeadlineGroup(
        new TimedAutoDrive(drivetrainSubsystem, new ChassisSpeeds(1, 0, 0), 2, false),
        new InstantCommand(intakeSubsystem::intakeBall)
        ),
      new InstantCommand(intakeSubsystem::stopIntake)
      .andThen(() -> { if(intakeSubsystem.pistonsForward) intakeSubsystem.retractIntake();}),
      new AimAndFire(shooterSubsystem, conveyorSubsystem, limelight, drivetrainSubsystem)
    );
  }
}
