package frc.robot.commands.Deprecated;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ColorSensor;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.Constants.*;

public class MoveConveyor extends CommandBase {

    private ConveyorSubsystem conveyorSubsystem;
    private ColorSensor colorSensor;
    
    public MoveConveyor(ConveyorSubsystem conveyorSubsystem, ColorSensor colorSensor){
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
