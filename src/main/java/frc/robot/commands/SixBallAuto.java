package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Constants.TrajectoryPaths;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.Limelight;
import frc.robot.Constants;

public class SixBallAuto extends SequentialCommandGroup {
    public SixBallAuto(IntakeSubsystem intakeSubsystem, Limelight limelight, DrivetrainSubsystem drivetrainSubsystem) {
        new ParallelCommandGroup(
                new Paths(frc.robot.Constants.TrajectoryPaths.BACK_UP_TO_NEXT_BALL, drivetrainSubsystem),
                new IntakeBall(intakeSubsystem),
                new AimbotPID(limelight, drivetrainSubsystem),
                new Paths(frc.robot.Constants.TrajectoryPaths.TURN_1, drivetrainSubsystem),
                new ParallelCommandGroup(
                        new Paths(frc.robot.Constants.TrajectoryPaths.COLLECT_NEAR_HUMAN_PLAYER, drivetrainSubsystem),
                        new IntakeBall(intakeSubsystem)),
                new Paths(frc.robot.Constants.TrajectoryPaths.TURN_2, drivetrainSubsystem),
                new AimbotPID(limelight, drivetrainSubsystem),
                new Paths(frc.robot.Constants.TrajectoryPaths.TURN_3, drivetrainSubsystem),
                new ParallelCommandGroup(
                        new Paths(frc.robot.Constants.TrajectoryPaths.BACK_UP_TO_NEXT_BALL, drivetrainSubsystem),
                        new IntakeBall(intakeSubsystem)),
                new Paths(frc.robot.Constants.TrajectoryPaths.TURN_4, drivetrainSubsystem),
                new ParallelCommandGroup(
                        new Paths(frc.robot.Constants.TrajectoryPaths.GET_FINAL_BALL, drivetrainSubsystem),
                        new IntakeBall(intakeSubsystem)),
                new Paths(frc.robot.Constants.TrajectoryPaths.TURN_5, drivetrainSubsystem),
                new AimbotPID(limelight, drivetrainSubsystem));
    }
}
