package frc.robot.commands;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class Shoot extends InstantCommand {
    ShooterSubsystem shooter;
    private double power;
    
    public Shoot(ShooterSubsystem shooter , double power) {
        this.shooter = shooter;
        this.power = power;
        addRequirements(shooter);
    }

    @Override
    public void execute() {
        shooter.rampUpShooter(power);
    }


}