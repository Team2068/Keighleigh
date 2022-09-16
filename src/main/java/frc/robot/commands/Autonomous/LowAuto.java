package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class LowAuto extends SequentialCommandGroup {
    //running a sequential command group so each command
    //will run one at a time
    public LowAuto(ShooterSubsystem shooterSubsystem, ConveyorSubsystem conveyorSubsystem) {
        addCommands(
            new InstantCommand(() -> shooterSubsystem.setRPM(1600)).alongWith(new WaitCommand(2)),
            new InstantCommand(conveyorSubsystem::moveConveyor).alongWith(new WaitCommand(2))
            .andThen(conveyorSubsystem::stopConveyor)
            .andThen(shooterSubsystem::rampDownShooter) 
            );
        //the auto should ramp up the shooter to 1600rpm
        //then it should run the conveyor to spit out the preload 
        //into the lower hub.
        //then the robot should stop
    }

}
