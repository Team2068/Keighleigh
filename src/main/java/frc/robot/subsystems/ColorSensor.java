// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.ColorSensorV3;

public class ColorSensor extends SubsystemBase {

  I2C.Port port1 = I2C.Port.kOnboard;
  I2C.Port port2 = I2C.Port.kMXP;

  ColorSensorV3 colorSensorLower = new ColorSensorV3(port1);
  ColorSensorV3 colorSensorUpper = new ColorSensorV3(port2);

  private double proxyCheck = 0;

  public boolean occupiedLower(){
    return (colorSensorLower.getProximity() > proxyCheck);
  }

  public boolean occupiedUpper() {
    return (colorSensorUpper.getProximity() > proxyCheck);
  }

  public ColorSensor() { }
}
