package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.HangConstants;
import frc.robot.subsystems.*;
public class RetractHangSubsystem extends CommandBase{
    private HangSubsystem hangSubsystem;
    
    public RetractHangSubsystem(HangSubsystem hangSubsystem){

        this.hangSubsystem = hangSubsystem;
      
        addRequirements(hangSubsystem);
    }
    @Override
    public void execute(){
        hangSubsystem.RetractHangSubsystem();
    }

    @Override
    public void end(boolean interrupted) {
        hangSubsystem.StopHang();
    }
}
