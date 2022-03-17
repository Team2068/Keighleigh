package frc.robot.commands;

import java.nio.file.Path;
import frc.robot.Constants;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.*;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.DrivetrainSubsystem;
public class DrivePaths extends CommandBase{
    public DrivePaths(Trajectory trajectory){
    DrivetrainSubsystem drivetrainSubsystem;
    PathPlannerTrajectory BackUpToCollectBall = PathPlanner.loadPath("BackUpToCollectBall", 1, 1);
    PathPlannerTrajectory Turn1 = PathPlanner.loadPath("Turn1", 1, 1);
    PathPlannerTrajectory CollectNearHumanPlayer = PathPlanner.loadPath("CollectNearHumanPlayer", 1, 1);
    PathPlannerTrajectory Turn2 = PathPlanner.loadPath("Turn2", 1, 1);
    PathPlannerTrajectory Turn3 = PathPlanner.loadPath("Turn3", 1, 1);
    PathPlannerTrajectory BackUpToNextBall = PathPlanner.loadPath("BackUpToNextBall", 1, 1);
    PathPlannerTrajectory Turn4 = PathPlanner.loadPath("Turn4", 1, 1);
    PathPlannerTrajectory GetFinalBall = PathPlanner.loadPath("GetFinalBall", 1, 1);
    PathPlannerTrajectory Turn5 = PathPlanner.loadPath("Turn5", 1, 1);
    PPSwerveControllerCommand command = new PPSwerveControllerCommand(BackUpToCollectBall,
    Turn1, CollectNearHumanPlayer, Turn2, Turn3, BackUpToNextBall, Turn4, GetFinalBall, Turn5,
     drivetrainSubsystem.getPose(),
        drivetrainSubsystem.m_kinematics, 
        new PIDController(AutoConstants.kPXController, 0, 0), new PIDController(AutoConstants.kPYController, 0, 0), new PIDController(AutoConstants.kPThetaController, 0, 0), 
      //pass in drive modules output
       drivetrainSubsystem.setModuleStates(states) ,drivetrainSubsystem);
         
        
         
    }
}
