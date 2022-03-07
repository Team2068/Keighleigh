package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.HangSubsystem;
import frc.robot.Constants.HangConstants;
public class ExtendHangSubsystem extends InstantCommand {
   
    private HangSubsystem hangSubsystem;
    
    public ExtendHangSubsystem(HangSubsystem hangSubsystem){
        this.hangSubsystem = hangSubsystem;
        addRequirements(hangSubsystem);
    }
    
    @Override
    public void initialize(){
   
        hangSubsystem.ExtendHangSubsystem(HangConstants.HANG_SPEED);
    }
}
