package frc.robot.commands.Mechanisms;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.Constants.*;

public class MoveConveyor extends CommandBase {

    private ConveyorSubsystem conveyorSubsystem;
    
    public MoveConveyor(ConveyorSubsystem conveyorSubsystem){
        this.conveyorSubsystem = conveyorSubsystem;
        addRequirements(conveyorSubsystem);
    }
    @Override
    public void execute(){
        conveyorSubsystem.moveConveyor(ConveyorConstants.CONVEYOR_SPEED);
    }

    @Override
    public void end(boolean interrupted) {
        conveyorSubsystem.stopConveyor();
    }
}
