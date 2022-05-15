package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Deprecated.ShooterOff;
import frc.robot.commands.AimShotPID;
import frc.robot.commands.Mechanisms.MoveConveyor;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.ShooterSubsystem;

public class HighAuto extends SequentialCommandGroup {

    public HighAuto(ShooterSubsystem shooterSubsystem, ConveyorSubsystem conveyorSubsystem, Limelight limelight) {
        addCommands(
            new AimShotPID(shooterSubsystem, 3900).withTimeout(2),
            new MoveConveyor(conveyorSubsystem).withTimeout(2),
            new ShooterOff(shooterSubsystem));
    }

}
