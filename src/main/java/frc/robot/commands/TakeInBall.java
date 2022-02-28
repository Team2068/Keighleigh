package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.Constants.*;

public class TakeInBall extends InstantCommand {
    private ConveyorSubsystem conveyorSubsystem;
    private IntakeSubsystem intakeSubsystem;
    
    public TakeInBall(ConveyorSubsystem conveyorSubsystem, IntakeSubsystem intakeSubsystem){
        this.conveyorSubsystem = conveyorSubsystem;
        this.intakeSubsystem = intakeSubsystem;
      
        addRequirements(conveyorSubsystem, intakeSubsystem);
    }
    @Override
    public void initialize(){
        conveyorSubsystem.takeInBall(ConveyorConstants.CONVEYOR_SPEED);
        intakeSubsystem.intakeBall(IntakeConstants.INTAKE_SPEED);
    }
}
