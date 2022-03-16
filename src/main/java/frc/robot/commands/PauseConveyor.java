package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.commands.Deprecated.MoveConveyor;

public class PauseConveyor extends SequentialCommandGroup {

    private ConveyorSubsystem conveyorSubsystem;
    
    public PauseConveyor(ConveyorSubsystem conveyorSubsystem){
        this.conveyorSubsystem = conveyorSubsystem;
        addRequirements(conveyorSubsystem);
    }
    
    @Override
    public void execute(){
        addCommands(new WaitCommand(2), new MoveConveyor(conveyorSubsystem));
    }
}
