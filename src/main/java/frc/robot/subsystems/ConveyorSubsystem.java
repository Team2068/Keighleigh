package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ConveyorConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class ConveyorSubsystem extends SubsystemBase {
    private CANSparkMax lowerConveyor = new CANSparkMax(ConveyorConstants.LOWER_CONVEYOR, MotorType.kBrushless);

    public ConveyorSubsystem() {
        lowerConveyor.setIdleMode(IdleMode.kCoast);
    }

    public void moveConveyor(double speed) {
        lowerConveyor.set(speed);
    }

    public void stopConveyor() {
        lowerConveyor.set(0);
    }
}
