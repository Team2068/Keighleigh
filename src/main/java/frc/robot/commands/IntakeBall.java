package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.Constants.*;

public class IntakeBall extends InstantCommand {
    private IntakeSubsystem intakeSubsystem;
    
    public IntakeBall(IntakeSubsystem intakeSubsystem){
        this.intakeSubsystem = intakeSubsystem;
      
        addRequirements(intakeSubsystem);
    }
    @Override
    public void initialize(){
        intakeSubsystem.intakeBall(IntakeConstants.INTAKE_SPEED);
    }
}
