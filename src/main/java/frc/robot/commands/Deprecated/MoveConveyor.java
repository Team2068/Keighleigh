package frc.robot.commands.Deprecated;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.Constants.*;

public class MoveConveyor extends CommandBase {
    // private IntakeSubsystem intakeSubsystem;
    private ConveyorSubsystem conveyorSubsystem;
    
    public MoveConveyor(ConveyorSubsystem conveyorSubsystem){
        this.conveyorSubsystem = conveyorSubsystem;
        addRequirements(conveyorSubsystem);
    }
    @Override
    public void initialize(){
        conveyorSubsystem.moveConveyor(ConveyorConstants.CONVEYOR_SPEED);
    }
}
