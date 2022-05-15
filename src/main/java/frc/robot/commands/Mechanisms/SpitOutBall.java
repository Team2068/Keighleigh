package frc.robot.commands.Mechanisms;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.Constants.ConveyorConstants;
import frc.robot.Constants.IntakeConstants;

public class SpitOutBall extends CommandBase{
    private IntakeSubsystem  intakeSubsystem;
    private ConveyorSubsystem conveyorSubsystem;

    public SpitOutBall(IntakeSubsystem intakeSubsystem, ConveyorSubsystem conveyorSubsystem){
        this.intakeSubsystem = intakeSubsystem;
        this.conveyorSubsystem = conveyorSubsystem;
        addRequirements(intakeSubsystem);
    }
    @Override
    public void initialize(){
        intakeSubsystem.moveIntake(IntakeConstants.SPIT_OUT_BALL);
        conveyorSubsystem.moveConveyor(ConveyorConstants.CONVEYOR_SPEED * -1);
    }

    @Override
    public void end(boolean interrupted) {
      conveyorSubsystem.stopConveyor();
      intakeSubsystem.stopIntake();
    }
}
