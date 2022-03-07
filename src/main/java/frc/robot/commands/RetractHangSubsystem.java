package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.HangConstants;
import frc.robot.subsystems.*;
public class RetractHangSubsystem extends InstantCommand{
    private HangSubsystem hangSubsystem;
    
    public RetractHangSubsystem(HangSubsystem hangSubsystem){

        this.hangSubsystem = hangSubsystem;
      
        addRequirements(hangSubsystem);
    }
    @Override
    public void initialize(){
   
        hangSubsystem.RetractHangSubsystem(HangConstants.HANG_SPEED);
    }
}
