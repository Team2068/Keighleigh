// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase {
  // Diet Limelight | remove entire section during merge
  public Limelight() {}

  private TargetData targetData = new TargetData();

  public class TargetData {
    public double horizontalOffset = 0; // Horizontal Offset From Crosshair To Target -29.8 to 29.8 degrees
  }

  public TargetData getTargetData(){
  return targetData;
  }
}
