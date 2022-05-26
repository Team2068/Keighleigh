package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ConveyorConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class ConveyorSubsystem extends SubsystemBase {
    private CANSparkMax lowerConveyor = new CANSparkMax(ConveyorConstants.LOWER_CONVEYOR, MotorType.kBrushless);

    private NetworkTableEntry blueDetected1 = NetworkTableInstance.getDefault().getEntry("blue");
    private NetworkTableEntry redDetected1 = NetworkTableInstance.getDefault().getEntry("red");
    private NetworkTableEntry greenDetected1 = NetworkTableInstance.getDefault().getEntry("green");

    private Color colorSensor1 = new Color(0,0,0);


    public ConveyorSubsystem() {
        lowerConveyor.setIdleMode(IdleMode.kCoast);
    }

    public void moveConveyor(double speed) {
        lowerConveyor.set(speed);
    }

    public void stopConveyor() {
        lowerConveyor.set(0);
    }

    public boolean isBallRed() {
        return colorSensor1.red >= colorSensor1.blue * 5.0;
    }

    public boolean isBallBlue() {
        return colorSensor1.blue >= colorSensor1.red * 3.0;
    }

    @Override
    public void periodic() {
        colorSensor1 = new Color(redDetected1.getDouble(0), greenDetected1.getDouble(0), blueDetected1.getDouble(0));
    }
}
