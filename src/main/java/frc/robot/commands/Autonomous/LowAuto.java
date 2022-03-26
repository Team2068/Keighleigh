package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AimShotPID;
import frc.robot.commands.Deprecated.MoveConveyor;
import frc.robot.commands.Deprecated.ShooterOff;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class LowAuto extends SequentialCommandGroup {

    public LowAuto(ShooterSubsystem shooterSubsystem, ConveyorSubsystem conveyorSubsystem) {
        addCommands(
            new AimShotPID(shooterSubsystem, 1600).withTimeout(2),
            new MoveConveyor(conveyorSubsystem).withTimeout(2),
            new ShooterOff(shooterSubsystem)
            );
    }

}
