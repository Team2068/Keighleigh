// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands.Autonomous;

// import edu.wpi.first.math.kinematics.ChassisSpeeds;
// import edu.wpi.first.wpilibj2.command.ConditionalCommand;
// import edu.wpi.first.wpilibj2.command.InstantCommand;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import frc.robot.Constants.ConveyorConstants;
// import frc.robot.subsystems.ColorSensor;
// import frc.robot.subsystems.ConveyorSubsystem;
// import frc.robot.subsystems.DrivetrainSubsystem;
// import frc.robot.subsystems.IntakeSubsystem;
// import frc.robot.subsystems.LidarSubsystem;
// import frc.robot.subsystems.Limelight;
// import frc.robot.subsystems.ShooterSubsystem;

// public class Brain extends SequentialCommandGroup {

//   public Brain(ShooterSubsystem shooterSubsystem, ConveyorSubsystem conveyorSubsystem,
//       DrivetrainSubsystem drivetrainSubsystem,
//       Limelight limelight, LidarSubsystem lidarSubsystem, IntakeSubsystem intakeSubsystem, ColorSensor colorSensor) {
//     addCommands(
//         new SequentialCommandGroup(
//             new LowAuto(shooterSubsystem, conveyorSubsystem),
//             new TimedAutoDrive(drivetrainSubsystem, new ChassisSpeeds(3, 0, 0), 1.5)),
//         new ConditionalCommand(
//             // Replace with AimAndFire
//             new InstantCommand(
//               () -> limelight.setPipeline(0)
//             ),
//             new ConditionalCommand(
//                 // Search
//                 new InstantCommand(() -> 
//                 {
//                   //Red Balls {Alliances will be later}
//                   limelight.setPipeline(1);
//                   drivetrainSubsystem.drive(new ChassisSpeeds(0, 0, 0.25));
//                 }),
//                 new SequentialCommandGroup(
//                     new InstantCommand(
//                         () -> {
//                           if (colorSensor.occupiedLower())
//                             while (!colorSensor.occupiedUpper())
//                               conveyorSubsystem.moveConveyor(ConveyorConstants.CONVEYOR_SPEED);
//                         }),
//                     // Pursure and Catch
//                     new Capture(limelight, lidarSubsystem, drivetrainSubsystem, intakeSubsystem, colorSensor)),
//                 () -> limelight.getTargetData().horizontalOffset == 0),
//             () -> colorSensor.occupiedLower() && colorSensor.occupiedUpper()));
//   }
// }
