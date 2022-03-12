package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.ColorSensor;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.Constants.*;
import frc.robot.commands.Deprecated.MoveConveyor;

public class PauseConveyor extends SequentialCommandGroup {

    private ConveyorSubsystem conveyorSubsystem;
    private ColorSensor colorSensor;
    
    public PauseConveyor(ConveyorSubsystem conveyorSubsystem, ColorSensor colorSensor){
        this.conveyorSubsystem = conveyorSubsystem;
        addRequirements(conveyorSubsystem);
    }
    @Override
    public void execute(){
        addCommands(new WaitCommand(2000), new MoveConveyor(conveyorSubsystem, colorSensor));
    }


}
