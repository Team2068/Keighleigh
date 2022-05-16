package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ConveyorConstants;
import frc.robot.Deprecated.RobotState;
import frc.robot.subsystems.ConveyorSubsystem;

public class AdjustConveyor extends CommandBase {
  ConveyorSubsystem conveyorSubsystem;

  public AdjustConveyor(ConveyorSubsystem conveyorSubsystem) {
    addRequirements(conveyorSubsystem);
    this.conveyorSubsystem = conveyorSubsystem;
  }

  @Override
  public void execute() {
    if (RobotState.getEntryValue("Sensors", "Upper Occupied").getBoolean()) // you can turn it into a variable if readablity is a problem [Iida]
      conveyorSubsystem.moveConveyor(ConveyorConstants.CONVEYOR_SPEED * -1);
  }

  @Override
  public void end(boolean interrupted) {
    conveyorSubsystem.stopConveyor();
  }

  @Override
  public boolean isFinished() {
  // This definitely is a readability nightmare
  return RobotState.getEntryValue("Sensors", "Upper Occupied").getBoolean() && !RobotState.getEntryValue("Sensors", "Lower Occupied").getBoolean();
  }
}
