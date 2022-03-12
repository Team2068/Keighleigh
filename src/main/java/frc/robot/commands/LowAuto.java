package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.Deprecated.MoveConveyor;
import frc.robot.subsystems.ColorSensor;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class LowAuto extends SequentialCommandGroup {

    private ShooterSubsystem shooterSubsystem;
    private ConveyorSubsystem conveyorSubsystem;
    private ColorSensor colorSensor;
    
    public LowAuto(ShooterSubsystem shooterSubsystem, ConveyorSubsystem conveyorSubsystem) {
        this.shooterSubsystem = shooterSubsystem;
        this.conveyorSubsystem = conveyorSubsystem;
        

        addCommands(
            new AimShotPID(shooterSubsystem, 1300), 
            new MoveConveyor(conveyorSubsystem, colorSensor));
    }

}
