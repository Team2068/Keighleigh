package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.*;
public class AutonTest1 extends SequentialCommandGroup{
   public AutonTest1(){
       super(
           new DrivePaths()
       )
   } 
}
