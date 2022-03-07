package frc.robot.commands.Deprecated;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterOff extends InstantCommand {
    ShooterSubsystem shooter;
    public ShooterOff(ShooterSubsystem shooter) {
        this.shooter = shooter;
        addRequirements(shooter);
    }

    @Override
    public void execute() {
        shooter.rampDownShooter();
    }
}