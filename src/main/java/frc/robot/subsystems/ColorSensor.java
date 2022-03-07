// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.ColorSensorV3;

public class ColorSensor extends SubsystemBase {

  I2C.Port port = I2C.Port.kOnboard;

  ColorSensorV3 colorSensor = new ColorSensorV3(port);

  public double InitialProximity;
  private double proxyCheck = 0;

  public double GetProximity(){
    return colorSensor.getProximity();
  }

  public boolean Occupied(){
    return (colorSensor.getProximity() < proxyCheck);
  }

  public ColorSensor() {
    InitialProximity = colorSensor.getProximity();
  }
}
