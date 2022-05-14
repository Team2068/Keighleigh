package frc.robot.commands.Mechanisms;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.Constants.*;

public class ReverseIntake extends CommandBase {
    private IntakeSubsystem intakeSubsystem;
    
    public ReverseIntake(IntakeSubsystem intakeSubsystem){
        this.intakeSubsystem = intakeSubsystem;
      
        addRequirements(intakeSubsystem);
    }
    @Override
    public void initialize(){
        intakeSubsystem.moveIntake(IntakeConstants.SPIT_OUT_BALL);
    }
    @Override
    public void end(boolean interrupted){
        intakeSubsystem.stopIntake();
    }
}
