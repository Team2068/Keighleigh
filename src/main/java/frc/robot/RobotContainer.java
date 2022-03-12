// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import javax.sound.sampled.Control;

import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.LimelightConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.AimShotCalculated;
import frc.robot.commands.AimShotPID;
import frc.robot.commands.AimbotPID;
// import frc.robot.commands.ControlIntakeSolenoids;
// import frc.robot.commands.ControlIntakeSolenoids;
//import frc.robot.commands.ControlIntakeSolenoids;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.ExtendHangSubsystem;
import frc.robot.commands.RetractHangSubsystem;
import frc.robot.commands.Shoot;
import frc.robot.commands.SpitOutBall;
import frc.robot.commands.TakeInBall;
import frc.robot.commands.Deprecated.IntakeBall;
import frc.robot.commands.Deprecated.MoveConveyor;
import frc.robot.commands.Deprecated.ReverseIntake;
import frc.robot.subsystems.ColorSensor;
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
  private final ColorSensor colorSensor = new ColorSensor();
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
        () -> modifyAxis(driverController.getLeftX()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
        () -> modifyAxis(driverController.getRightX()) * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND
    ));

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
    JoystickButton mechBumperL = new JoystickButton(mechanismController, Button.kLeftBumper.value);
    JoystickButton driveBumperL = new JoystickButton(driverController, Button.kLeftBumper.value);
    JoystickButton driveBumperR = new JoystickButton(driverController, Button.kRightBumper.value);
    JoystickButton driverA = new JoystickButton(driverController, Button.kA.value);
    JoystickButton driverB = new JoystickButton(driverController, Button.kB.value);
    JoystickButton driverY = new JoystickButton(driverController, Button.kY.value);
    JoystickButton driverX = new JoystickButton(driverController, Button.kX.value);

    // Back button zeros the gyroscope

    // mechBumperR.whileActiveContinuous(new TakeInBall(conveyorSubsystem, intakeSubsystem));
    // mechRightTrigger.whileActiveContinuous(new SpitOutBall(intakeSubsystem, conveyorSubsystem));

    //mechBumperL.whileHeld(new Shoot(shooterSubsystem, 0.8));
    mechBumperR.whileHeld(new IntakeBall(intakeSubsystem));
    mechRightTrigger.whileActiveContinuous(new MoveConveyor(conveyorSubsystem, colorSensor));
    mechLeftTrigger.whileActiveContinuous(new SpitOutBall(intakeSubsystem, conveyorSubsystem));
    mechY.whileHeld(new ReverseIntake(intakeSubsystem));
    // mechA.whileHeld(new AimbotPID(limelight, drivetrainSubsystem));
    // mechB.whileHeld(new AimShot(shooterSubsystem, limelight));
    //driverX.whenPressed(new ConditionalCommand(new InstantCommand(), new AimShotPID(shooterSubsystem, 2500), () -> new WaitCommand(5).isFinished()));
    mechBumperL.whenHeld(new AimShotCalculated(shooterSubsystem, limelight)).whenInactive(() -> shooterSubsystem.rampDownShooter());
    mechX.whenHeld(new AimShotPID(shooterSubsystem, ShooterConstants.LOWER_HUB_RPM), true).whenInactive(() -> shooterSubsystem.rampDownShooter());
    mechB.whenHeld(new AimShotPID(shooterSubsystem, ShooterConstants.UPPER_HUB_FALLBACK_RPM), true).whenInactive(() -> shooterSubsystem.rampDownShooter());

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
    // An ExampleCommand will run in autonomous
    return new InstantCommand();
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
