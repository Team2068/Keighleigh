package frc.robot.subsystems;

import frc.robot.Constants.*;
// import frc.robot.RobotContainer;
// import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.DigitalInput;
// import edu.wpi.first.wpilibj.DoubleSolenoid;
// import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants;
import com.revrobotics.*;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class ConveyorSubsystem extends SubsystemBase {
    private DigitalInput toplimitSwitch = new DigitalInput(0);
    private CANSparkMax lowerConveyor = new CANSparkMax(ConveyorConstants.LOWER_CONVEYOR, MotorType.kBrushless);
   // private CANSparkMax upperConveyor = new CANSparkMax(ConveyorConstants.UPPER_CONVEYOR, MotorType.kBrushless);

    public ConveyorSubsystem() {
        lowerConveyor.setIdleMode(IdleMode.kCoast);
        //upperConveyor.setIdleMode(IdleMode.kCoast);
    }

    public void moveConveyor(double speed) {
        lowerConveyor.set(speed);
       // upperConveyor.set(speed);
    }

    public void stopConveyor() {
        //upperConveyor.set(0);
        lowerConveyor.set(0);
    }

    public void takeInBall(double speed) {
        if (toplimitSwitch.get()) {
            stopConveyor();
        } else {
           // upperConveyor.set(speed);
            lowerConveyor.set(speed);
        }
    }
}
