package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.Deprecated.MoveConveyor;
import frc.robot.commands.Deprecated.ShooterOff;
import frc.robot.subsystems.ColorSensor;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;

public class AimAndFire extends SequentialCommandGroup {

    public AimAndFire(ShooterSubsystem shooterSubsystem, ConveyorSubsystem conveyorSubsystem, Limelight limelight, ColorSensor colorSensor, DrivetrainSubsystem drivetrainSubsystem) {
        addCommands(
            //new AdjustConveyor(conveyorSubsystem, colorSensor),
            new AimBotAngle(limelight, drivetrainSubsystem).withTimeout(2),
            new AimShotCalculated(shooterSubsystem, limelight),
            new WaitCommand(1),
            new MoveConveyor(conveyorSubsystem).withTimeout(2),
            new ShooterOff(shooterSubsystem));
    }
}
