package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.Mechanisms.MoveConveyor;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;

public class AimAndFire extends SequentialCommandGroup {

    public AimAndFire(ShooterSubsystem shooterSubsystem, ConveyorSubsystem conveyorSubsystem, Limelight limelight, DrivetrainSubsystem drivetrainSubsystem) {
        addCommands(
            //new AdjustConveyor(conveyorSubsystem, colorSensor),
            new ParallelDeadlineGroup(new AimBotAngle(limelight, drivetrainSubsystem).withTimeout(0.7), 
                new InstantCommand(() -> shooterSubsystem.setRPM(3500))),
            new InstantCommand(() -> shooterSubsystem.setRPM(limelight.lerpRPM())),
            new WaitCommand(0.2),
            new MoveConveyor(conveyorSubsystem).withTimeout(1.25),
            new InstantCommand(() -> shooterSubsystem.rampDownShooter()));
    }
}
