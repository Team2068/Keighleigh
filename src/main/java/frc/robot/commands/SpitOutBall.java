package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.Constants.ConveyorConstants;

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
        intakeSubsystem.reverseIntake();
        conveyorSubsystem.moveConveyor(ConveyorConstants.CONVEYOR_SPEED * -1);
    }

    @Override
    public void end(boolean interrupted) {
      conveyorSubsystem.stopConveyor();
      intakeSubsystem.stopIntake();
    }
}
