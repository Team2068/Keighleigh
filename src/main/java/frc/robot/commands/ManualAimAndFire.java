package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Constants.ConveyorConstants;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;

public class ManualAimAndFire extends SequentialCommandGroup {

    public ManualAimAndFire(ShooterSubsystem shooterSubsystem, ConveyorSubsystem conveyorSubsystem, Limelight limelight, DrivetrainSubsystem drivetrainSubsystem) {
        addCommands(
            //new AdjustConveyor(conveyorSubsystem, colorSensor),
            new ParallelDeadlineGroup(new AimBotAngle(limelight, drivetrainSubsystem).withTimeout(0.7), new InstantCommand(() -> shooterSubsystem.setRPM(1500)))
            .andThen(() -> {
                // System.out.printf("[AimAndFire] RPM: %f\n", rpm);
                shooterSubsystem.setRPM(Constants.ShooterConstants.UPPER_HUB_FALLBACK_RPM);
            }),
            new WaitCommand(0.25),
            new InstantCommand(() -> conveyorSubsystem.moveConveyor(ConveyorConstants.CONVEYOR_SPEED))
                .alongWith(new WaitCommand(1.5))
                .andThen(conveyorSubsystem::stopConveyor)
                .andThen(shooterSubsystem::rampDownShooter)
            );
    }
}
