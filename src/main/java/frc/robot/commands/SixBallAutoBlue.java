package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.ShooterSubsystem;

public class SixBallAutoBlue extends SequentialCommandGroup {
  public SixBallAutoBlue(IntakeSubsystem intakeSubsystem, Limelight limelight, DrivetrainSubsystem drivetrainSubsystem,
      ShooterSubsystem shooterSubsystem, ConveyorSubsystem conveyorSubsystem) {
    super(
        new SequentialCommandGroup(
            new InstantCommand(intakeSubsystem::deployIntake),
            new ParallelCommandGroup(
                new Paths(frc.robot.Constants.TrajectoryPaths.BackToCollectBall, drivetrainSubsystem),
                new InstantCommand(intakeSubsystem::intakeBall)),
            new AimAndFire(shooterSubsystem, conveyorSubsystem, limelight, drivetrainSubsystem),
            new InstantCommand(intakeSubsystem::retractIntake),
            new Paths(frc.robot.Constants.TrajectoryPaths.Step1, drivetrainSubsystem),
            new InstantCommand(intakeSubsystem::deployIntake),
            new WaitCommand(2),
            new InstantCommand(intakeSubsystem::intakeBall),
            new AimAndFire(shooterSubsystem, conveyorSubsystem, limelight, drivetrainSubsystem)));
            // new ParallelCommandGroup(
            //     new Paths(frc.robot.Constants.TrajectoryPaths.Step2, drivetrainSubsystem),
            //     new InstantCommand(intakeSubsystem::intakeBall)),
            // new ParallelCommandGroup(
            //     new Paths(frc.robot.Constants.TrajectoryPaths.Step3, drivetrainSubsystem),
            //     new InstantCommand(intakeSubsystem::intakeBall)),
            // new AimAndFire(shooterSubsystem, conveyorSubsystem, limelight, drivetrainSubsystem)));
  }
}
