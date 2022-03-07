package frc.robot.commands.Deprecated;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ConveyorSubsystem;

public class StopConveyor extends InstantCommand {
    private ConveyorSubsystem conveyorSubsystem;
    
    public StopConveyor(ConveyorSubsystem conveyorSubsystem){
        this.conveyorSubsystem = conveyorSubsystem;
      
        addRequirements(conveyorSubsystem);
    }
    @Override
    public void initialize(){
        conveyorSubsystem.stopConveyor();
    }
}
