package frc.robot.commands.Deprecated;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.Constants.*;

public class IntakeBall extends InstantCommand {
    //private ConveyorSubsystem conveyorSubsystem;
    private IntakeSubsystem intakeSubsystem;
    
    public IntakeBall(IntakeSubsystem intakeSubsystem){
       // this.conveyorSubsystem = conveyorSubsystem;
        this.intakeSubsystem = intakeSubsystem;
      
        addRequirements(intakeSubsystem);
    }
    @Override
    public void initialize(){
        //conveyorSubsystem.takeInBall(ConveyorConstants.CONVEYOR_SPEED);
        intakeSubsystem.intakeBall(IntakeConstants.INTAKE_SPEED);
    }
}
