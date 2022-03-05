// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import javax.sound.sampled.Control;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
//import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.NetworkButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ControllerConstants;
// import frc.robot.commands.ControlIntakeSolenoids;
// import frc.robot.commands.ControlIntakeSolenoids;
//import frc.robot.commands.ControlIntakeSolenoids;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.ExtendHangSubsystem;
import frc.robot.commands.IntakeBall;
import frc.robot.commands.IntakeOff;
import frc.robot.commands.MoveConveyor;
import frc.robot.commands.RetractHangSubsystem;
import frc.robot.commands.ReverseIntake;
import frc.robot.commands.Shoot;
import frc.robot.commands.ShooterOff;
import frc.robot.commands.SpitOutBall;
import frc.robot.commands.StopConveyor;
import frc.robot.commands.TakeInBall;
import frc.robot.subsystems.ConveyorSubsystem;
// import frc.robot.commands.TakeInBall;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.HangSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
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

    //mechRightBumper.whileActiveContinuous(new MoveConveyor(conveyorSubsystem));
    mechA.whenPressed(new MoveConveyor(conveyorSubsystem));
    mechB.whenPressed(new StopConveyor(conveyorSubsystem));
    mechX.whenPressed(new IntakeBall(intakeSubsystem));
    mechY.whenPressed(new IntakeOff(intakeSubsystem));
    mechBumperR.whenPressed(new ReverseIntake(intakeSubsystem));
    mechBumperL.whenPressed(new SpitOutBall(intakeSubsystem, conveyorSubsystem));
    //driveBumperL.whenPressed(new ExtendHangSubsystem(hangSubsystem));
   // driveBumperR.whenPressed(new RetractHangSubsystem(hangSubsystem));
    driverA.whenPressed(new Shoot(shooterSubsystem, .8));
    driverB.whenPressed(new ShooterOff(shooterSubsystem));
    // driverY.whenPressed(new ControlIntakeSolenoids(intakeSubsystem));
    // new Button(driverController::getYButton)
    //     // No requirements because we don't need to interrupt anything
    //     .whenPressed(drivetrainSubsystem::zeroGyroscope);
    // // driver controller
    // // mechanism controller
    // new Button(mechanismController::getAButton).whileActiveContinuous(new IntakeBall(intakeSubsystem));
    // new Button(mechanismController::getRightBumper).whileActiveContinuous(new IntakeBall(intakeSubsystem));
    // mechLeftTrigger.whenActive(new SpitOutBall(intakeSubsystem, conveyorSubsystem));
    // mechRightTrigger.whenActive(new TakeInBall(conveyorSubsystem, intakeSubsystem));
    //new Button(mechanismController::getLeftBumper).whenActive(new MoveConveyor(conveyorSubsystem));
    //new Button(mechanismController::getYButton).whenPressed(new ControlIntakeSolenoids(intakeSubsystem));
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
