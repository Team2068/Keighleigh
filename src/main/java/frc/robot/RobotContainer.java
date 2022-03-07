// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import frc.robot.Constants.AutoConstants;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.LimelightConstants;
// import frc.robot.commands.ControlIntakeSolenoids;
// import frc.robot.commands.ControlIntakeSolenoids;
//import frc.robot.commands.ControlIntakeSolenoids;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.ExtendHangSubsystem;
import frc.robot.commands.RetractHangSubsystem;
import frc.robot.commands.Shoot;
import frc.robot.commands.SpitOutBall;
import frc.robot.commands.TakeInBall;
// import frc.robot.commands.Deprecated.IntakeBall;
// import frc.robot.commands.Deprecated.IntakeOff;
// import frc.robot.commands.Deprecated.MoveConveyor;
// import frc.robot.commands.Deprecated.ReverseIntake;
// import frc.robot.commands.Deprecated.ShooterOff;
// import frc.robot.commands.Deprecated.StopConveyor;
import frc.robot.subsystems.ConveyorSubsystem;
// import frc.robot.commands.TakeInBall;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.HangSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.Limelight;
// import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DrivetrainSubsystem drivetrainSubsystem = new DrivetrainSubsystem();
  private final ConveyorSubsystem conveyorSubsystem = new ConveyorSubsystem();
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  //private final HangSubsystem hangSubsystem = new HangSubsystem();
  private final XboxController driverController = new XboxController(0);
  private final XboxController mechanismController = new XboxController(1);
  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  private final Limelight limelight = new Limelight(LimelightConstants.LedMode.DEFAULT, LimelightConstants.CamMode.VISION);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Set up the default command for the drivetrain.
    // The controls are for field-oriented driving:
    // Left stick Y axis -> forward and backwards movement
    // Left stick X axis -> left and right movement
    // Right stick X axis -> rotation
    drivetrainSubsystem.setDefaultCommand(new DefaultDriveCommand(
        drivetrainSubsystem,
        () -> modifyAxis(driverController.getLeftY()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
        () -> -modifyAxis(driverController.getLeftX()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND * -1,
        () -> -modifyAxis(driverController.getRightX()) * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND
            * -1));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
   * it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // set up triggers and such
    Trigger mechRightTrigger = new Trigger(() -> mechanismController
        .getRawAxis(ControllerConstants.RIGHT_TRIGGER) > ControllerConstants.TRIGGER_ACTIVATION_THRESHOLD);
    Trigger mechLeftTrigger = new Trigger(() -> mechanismController
        .getRawAxis(ControllerConstants.LEFT_TRIGGER) > ControllerConstants.TRIGGER_ACTIVATION_THRESHOLD);
    JoystickButton mechA = new JoystickButton(mechanismController, Button.kA.value);
    JoystickButton mechB = new JoystickButton(mechanismController, Button.kB.value);
    JoystickButton mechX = new JoystickButton(mechanismController, Button.kX.value);
    JoystickButton mechY = new JoystickButton(mechanismController, Button.kY.value);
    JoystickButton mechBumperR = new JoystickButton(mechanismController, Button.kRightBumper.value);
    JoystickButton mechBumperL = new JoystickButton(mechanismController, Button.kRightBumper.value);
    JoystickButton driveBumperL = new JoystickButton(driverController, Button.kLeftBumper.value);
    JoystickButton driveBumperR = new JoystickButton(driverController, Button.kRightBumper.value);
    JoystickButton driverA = new JoystickButton(driverController, Button.kA.value);
    JoystickButton driverB = new JoystickButton(driverController, Button.kB.value);
    JoystickButton driverY = new JoystickButton(driverController, Button.kY.value);

    // Back button zeros the gyroscope

    mechBumperL.whileHeld(new Shoot(shooterSubsystem, 0.8));
    mechBumperR.whileHeld(new TakeInBall(conveyorSubsystem, intakeSubsystem));
    mechRightTrigger.whileActiveContinuous(new SpitOutBall(intakeSubsystem, conveyorSubsystem));

    //driveBumperL.whenPressed(new ExtendHangSubsystem(hangSubsystem));
   // driveBumperR.whenPressed(new RetractHangSubsystem(hangSubsystem));
    // driverY.whenPressed(new ControlIntakeSolenoids(intakeSubsystem));
    // new Button(driverController::getYButton)
    //     // No requirements because we don't need to interrupt anything
    //     .whenPressed(drivetrainSubsystem::zeroGyroscope);
  }

  /**
   * Use this to pass thex autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    System.out.println("getAutonomousCommand");

    TrajectoryConfig config = new TrajectoryConfig(AutoConstants.MAX_Speed_MetersPerSecond,
        AutoConstants.MAX_Acceleration_MetersPerSecondSquared)
            .setKinematics(m_drivetrainSubsystem.m_kinematics);

    var thetaController = new ProfiledPIDController(AutoConstants.kPThetaController, 0, 0,
        AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(3, 0, new Rotation2d(0)),
        // Pass config
        config);

    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(exampleTrajectory,
        m_drivetrainSubsystem::getPose, // Functional interface to feed supplier
        m_drivetrainSubsystem.m_kinematics,
        // Position controllers
        new PIDController(AutoConstants.kPXController, 0, 0), new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController, m_drivetrainSubsystem::setModuleStates, m_drivetrainSubsystem);

    // Reset odometry to the starting pose of the trajectory.
    m_drivetrainSubsystem.resetOdometryWithPose2d(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return swerveControllerCommand.andThen(() -> m_drivetrainSubsystem.drive(new ChassisSpeeds()));

    //  https://github.com/wpilibsuite/allwpilib/blob/main/wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/examples/swervecontrollercommand/subsystems/DriveSubsystem.java
  }

  private static double deadband(double value, double deadband) {
    if (Math.abs(value) > deadband) {
      if (value > 0.0) {
        return (value - deadband) / (1.0 - deadband);
      } else {
        return (value + deadband) / (1.0 - deadband);
      }
    } else {
      return 0.0;
    }
  }

  private static double modifyAxis(double value) {
    // Deadband
    value = deadband(value, 0.05);

    // Square the axis
    value = Math.copySign(value * value, value);

    return value;
  }
}
