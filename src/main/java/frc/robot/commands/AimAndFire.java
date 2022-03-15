package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Deprecated.MoveConveyor;
import frc.robot.commands.Deprecated.ShooterOff;
import frc.robot.subsystems.ColorSensor;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.ShooterSubsystem;

public class AimAndFire extends SequentialCommandGroup {
    public AimAndFire(ShooterSubsystem shooterSubsystem, ConveyorSubsystem conveyorSubsystem, Limelight limelight, ColorSensor colorSensor) {
        addCommands(
            new AdjustConveyor(conveyorSubsystem, colorSensor),
            new AimShotCalculated(shooterSubsystem, limelight).withTimeout(2),
            new MoveConveyor(conveyorSubsystem).withTimeout(2),
            //new IntakeBall incase ball gets stuck in intake
            new ShooterOff(shooterSubsystem));
    }

}
