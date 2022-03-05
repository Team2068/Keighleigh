package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.Constants.*;

public class ReverseIntake extends InstantCommand {
    //private ConveyorSubsystem conveyorSubsystem;
    private IntakeSubsystem intakeSubsystem;
    
    public ReverseIntake(IntakeSubsystem intakeSubsystem){
       // this.conveyorSubsystem = conveyorSubsystem;
        this.intakeSubsystem = intakeSubsystem;
      
        addRequirements(intakeSubsystem);
    }
    @Override
    public void initialize(){
        //conveyorSubsystem.takeInBall(ConveyorConstants.CONVEYOR_SPEED);
        intakeSubsystem.reverseIntake(IntakeConstants.SPIT_OUT_BALL);
    }
}
