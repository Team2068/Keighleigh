// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.TrajectoryPaths;
import frc.robot.commands.Autonomous.RedTwoBallHighGoal;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.ShooterSubsystem;

public class RedFourBallAuto extends SequentialCommandGroup {
  public RedFourBallAuto(IntakeSubsystem intakeSubsystem, Limelight limelight, DrivetrainSubsystem drivetrainSubsystem, ShooterSubsystem shooterSubsystem, ConveyorSubsystem conveyorSubsystem) {
    addCommands(
      new RedTwoBallHighGoal(intakeSubsystem, drivetrainSubsystem, shooterSubsystem, limelight, conveyorSubsystem),
      new InstantCommand(drivetrainSubsystem::zeroGyroscope),
      new InstantCommand(intakeSubsystem::controlIntakeSolenoids),
      new ParallelDeadlineGroup(
        new Paths(TrajectoryPaths.FourBallRed_GoToHumanPlayer, drivetrainSubsystem),
        new InstantCommand(intakeSubsystem::intakeBall)), // drive to le human player
      new WaitCommand(2),
      new ParallelDeadlineGroup(
        new Paths(TrajectoryPaths.FourBallRed_Little, drivetrainSubsystem),
        new InstantCommand(intakeSubsystem::controlIntakeSolenoids)),
      new Paths(TrajectoryPaths.FourBallRed_Shoot, drivetrainSubsystem), // drive to the shooting place thing
      new AimAndFire(shooterSubsystem, conveyorSubsystem, limelight, drivetrainSubsystem) // SHOOT AGAIN!!!!! :DDDD
    );
  }
}
