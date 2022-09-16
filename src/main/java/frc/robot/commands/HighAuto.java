package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.ShooterSubsystem;

public class HighAuto extends SequentialCommandGroup {

    public HighAuto(ShooterSubsystem shooterSubsystem, ConveyorSubsystem conveyorSubsystem, Limelight limelight) {
        addCommands(
            new InstantCommand(() -> shooterSubsystem.setRPM(3900)).alongWith(new WaitCommand(2)),
            new InstantCommand(conveyorSubsystem::moveConveyor).alongWith(new WaitCommand(2))
            .andThen(conveyorSubsystem::stopConveyor)
            .andThen(shooterSubsystem::rampDownShooter)
        );
    }

}
