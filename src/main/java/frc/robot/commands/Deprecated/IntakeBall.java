package frc.robot.commands.Deprecated;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.Constants.*;

public class IntakeBall extends CommandBase {
    private IntakeSubsystem intakeSubsystem;
    
    public IntakeBall(IntakeSubsystem intakeSubsystem){
        this.intakeSubsystem = intakeSubsystem;
      
        addRequirements(intakeSubsystem);
    }
    @Override
    public void execute(){
        intakeSubsystem.intakeBall(IntakeConstants.INTAKE_SPEED);
    }

    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.stopIntake();
    }
}
