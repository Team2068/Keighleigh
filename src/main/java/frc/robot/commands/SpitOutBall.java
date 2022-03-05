package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.Constants;
import frc.robot.Constants.ConveyorConstants;
import frc.robot.Constants.IntakeConstants;
public class SpitOutBall extends InstantCommand{
    private IntakeSubsystem  intakeSubsystem;
    private ConveyorSubsystem conveyorSubsystem;
    double spitOutBall = Constants.IntakeConstants.SPIT_OUT_BALL;

    public SpitOutBall(IntakeSubsystem intakeSubsystem, ConveyorSubsystem conveyorSubsystem){
        this.intakeSubsystem = intakeSubsystem;
        this.conveyorSubsystem = conveyorSubsystem;
        addRequirements(intakeSubsystem);
    }
    @Override
    public void initialize(){
        //reverse intake and conveyor at same time
        //intakeSubsystem.reverseIntake(spitOutBall);
        conveyorSubsystem.moveConveyor(ConveyorConstants.CONVEYOR_SPEED * -1);
    }
}
