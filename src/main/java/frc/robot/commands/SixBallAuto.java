package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Constants.TrajectoryPaths;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.Limelight;
import frc.robot.Constants;
public class SixBallAuto extends SequentialCommandGroup{
   public SixBallAuto(IntakeSubsystem intakeSubsystem, Limelight limelight, DrivetrainSubsystem drivetrainSubsystem){
    new ParallelCommandGroup(
    new Paths(frc.robot.Constants.TrajectoryPaths.BackUpToCollectBall,drivetrainSubsystem), new IntakeBall(intakeSubsystem),
    new AimbotPID(limelight, drivetrainSubsystem), 
    new Paths(frc.robot.Constants.TrajectoryPaths.Turn1,drivetrainSubsystem),
    new ParallelCommandGroup(
    new Paths(frc.robot.Constants.TrajectoryPaths.CollectNearHumanPlayer,drivetrainSubsystem), 
    new IntakeBall(intakeSubsystem)),
    new Paths(frc.robot.Constants.TrajectoryPaths.Turn2,drivetrainSubsystem),
    new AimbotPID(limelight, drivetrainSubsystem),
    new Paths(frc.robot.Constants.TrajectoryPaths.Turn3,drivetrainSubsystem),
    new ParallelCommandGroup(
        new Paths(frc.robot.Constants.TrajectoryPaths.BackUpToNextBall,drivetrainSubsystem),
        new IntakeBall(intakeSubsystem)),
         new Paths(frc.robot.Constants.TrajectoryPaths.Turn4,drivetrainSubsystem),
         new ParallelCommandGroup(
             new Paths(frc.robot.Constants.TrajectoryPaths.GetFinalBall,drivetrainSubsystem),
             new IntakeBall(intakeSubsystem)),
             new Paths(frc.robot.Constants.TrajectoryPaths.Turn5,drivetrainSubsystem), 
             new AimbotPID(limelight, drivetrainSubsystem));


   } 
}
