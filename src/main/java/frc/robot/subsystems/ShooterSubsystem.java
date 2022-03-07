// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;
import com.revrobotics.CANSparkMax;
import frc.robot.Constants.ShooterConstants;
import com.revrobotics.CANSparkMax.IdleMode;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class ShooterSubsystem extends SubsystemBase {

    CANSparkMax flywheel1 = new CANSparkMax(ShooterConstants.FLYWHEEL_1, MotorType.kBrushless);
    CANSparkMax flywheel2 = new CANSparkMax(ShooterConstants.FLYWHEEL_2, MotorType.kBrushless);

    public ShooterSubsystem() {
        flywheel1.setSmartCurrentLimit(Constants.CURRENT_LIMIT);
        flywheel2.setSmartCurrentLimit(Constants.CURRENT_LIMIT);

        flywheel1.setOpenLoopRampRate(.2);
        flywheel2.setOpenLoopRampRate(.2);

        flywheel1.setIdleMode(IdleMode.kCoast);
        flywheel2.setIdleMode(IdleMode.kCoast);
    }

    public void rampUpShooter(double speed) {
        flywheel1.set(-speed);
        flywheel2.set(speed);
    }

    public void rampDownShooter() {
        flywheel1.set(0);
        flywheel2.set(0);
    }

    public void setPower(double power) {
        flywheel1.setVoltage(power);
        flywheel2.setVoltage(power);
    }
}
