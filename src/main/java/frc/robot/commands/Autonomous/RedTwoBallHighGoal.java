package frc.robot.commands.Autonomous;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AimAndFire;
import frc.robot.commands.IntakeBall;
// import frc.robot.subsystems.ColorSensor;
import frc.robot.subsystems.*;

public class RedTwoBallHighGoal extends SequentialCommandGroup {
  /** Creates a new RedTwoBallHighGoal. */
  public RedTwoBallHighGoal(IntakeSubsystem intakeSubsystem, DrivetrainSubsystem drivetrainSubsystem, ShooterSubsystem shooterSubsystem, Limelight limelight, ConveyorSubsystem conveyorSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new InstantCommand(intakeSubsystem::toggleIntake),
      new ParallelDeadlineGroup(new TimedAutoDrive(drivetrainSubsystem, new ChassisSpeeds(1, 0, 0), 2, false), new IntakeBall(intakeSubsystem)),
      new InstantCommand(intakeSubsystem::toggleIntake),
      new InstantCommand(() -> { if(intakeSubsystem.pistonsForward) intakeSubsystem.retractIntake();}),
      new AimAndFire(shooterSubsystem, conveyorSubsystem, limelight, drivetrainSubsystem)
    );
  }
}
