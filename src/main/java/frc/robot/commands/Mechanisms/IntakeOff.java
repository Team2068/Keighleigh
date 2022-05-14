package frc.robot.commands.Mechanisms;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeOff extends InstantCommand {
    private IntakeSubsystem intakeSubsystem;
   // private ConveyorSubsystem conveyorSubsystem;
    
    public IntakeOff(IntakeSubsystem intakeSubsystem){
        this.intakeSubsystem = intakeSubsystem;
      
        addRequirements(intakeSubsystem);
    }
    @Override
    public void initialize(){
        intakeSubsystem.stopIntake();
    }
}
