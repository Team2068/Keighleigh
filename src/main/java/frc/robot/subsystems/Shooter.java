// package frc.robot.subsystems;

// import com.revrobotics.CANSparkMax;
// import com.revrobotics.CANSparkMax.IdleMode;
// import com.revrobotics.CANSparkMaxLowLevel.MotorType;

// import frc.robot.Constants.ShooterConstants;
// import frc.robot.Constants.GameElementConstants;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;

// public class Shooter extends SubsystemBase {
//     CANSparkMax flywheel1 = new CANSparkMax(ShooterConstants.FLYWHEEL_1, MotorType.kBrushless);
//     CANSparkMax flywheel2 = new CANSparkMax(ShooterConstants.FLYWHEEL_2, MotorType.kBrushless);

//     public Shooter() {
//         flywheel1.setOpenLoopRampRate(.2);
//         flywheel2.setOpenLoopRampRate(.2);

//         flywheel1.setIdleMode(IdleMode.kCoast);
//         flywheel2.setIdleMode(IdleMode.kCoast);
//     }

//     // distance will have to be a value measured by the limelight
//     // or a distance sensor. This will find the speed the flywheel
//     // has to start at in order to score. Will need to be converted to m/s.
//     public double findLowerSpeed(double distance) {
//         double vy = (GameElementConstants.LOW_HEIGHT + 11.03) / 1.5;
//         double vx = distance / 1.5;
//         double speed = Math.tan(vy / vx);
//         return speed;
//     }

//     // Will spin the motor until it gets up to speed.
//     public void rampUpShooter(double speed) {
//         flywheel1.set(-speed);
//         flywheel2.set(speed);
//     }

//     // Will let the motor coast to a stop
//     public void rampDownShooter() {
//         flywheel1.stopMotor();
//         flywheel2.stopMotor();
//         // flywheel1.set(0);
//         // flywheel2.set(0);
//     }

//     public void setPower(double power){
//         flywheel1.setVoltage(power);
//         flywheel2.setVoltage(power);
//     }
// }