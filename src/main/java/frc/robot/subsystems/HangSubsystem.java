package frc.robot.subsystems;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.HangConstants;
public class HangSubsystem extends SubsystemBase{
   
CANSparkMax LeftHangMotor = new CANSparkMax(HangConstants.LEFT_HANG_MOTOR, MotorType.kBrushless);
CANSparkMax RightHangMotor = new CANSparkMax(HangConstants.RIGHT_HANG_MOTOR, MotorType.kBrushless);

public HangSubsystem(){
    LeftHangMotor.setIdleMode(IdleMode.kCoast);
    LeftHangMotor.setSmartCurrentLimit(Constants.CURRENT_LIMIT);
    RightHangMotor.setIdleMode(IdleMode.kCoast);
    RightHangMotor.setSmartCurrentLimit(Constants.CURRENT_LIMIT);

}
public void ExtendHangSubsystem(double speed){
    LeftHangMotor.set(speed);
    RightHangMotor.set(speed);
}
public void RetractHangSubsystem(double speed){
LeftHangMotor.set(-speed);
RightHangMotor.set(-speed);
}

public void StopHang(){
    LeftHangMotor.set(0);
    RightHangMotor.set(0);
}
}
