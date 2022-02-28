package frc.robot.subsystems;

import frc.robot.Constants.*;
import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.revrobotics.*;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class IntakeSubsystem extends SubsystemBase {

    private DoubleSolenoid intakeSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH,
            IntakeConstants.FORWARD_CHANNEL, IntakeConstants.REVERSE_CHANNEL);
    private DigitalInput toplimitSwitch = new DigitalInput(0);
    CANSparkMax intake = new CANSparkMax(IntakeConstants.INTAKE_MOTOR, MotorType.kBrushless);

    public IntakeSubsystem() {
        intake.setIdleMode(IdleMode.kCoast);
    }

    public void reverseIntake(double speed) {
        intake.set(speed);
    }
    public void intakeBall(double speed) {
        intake.set(speed);
    }

    // public void TakeInBall(double Intakespeed) {
    //     if (Intakespeed > 0) {
    //         if (toplimitSwitch.get()) {
    //             intake.set(0);
    //         } else {

    //             intake.set(Intakespeed);
    //         }
    //     }
    // }
}
