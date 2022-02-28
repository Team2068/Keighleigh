package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.Constants.*;

public class MoveConveyor extends InstantCommand {
    private IntakeSubsystem intakeSubsystem;
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
