// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DigitalInput;


public class Test_MagLimit extends SubsystemBase {
  /** Creates a new Test_MagLimit. */
DigitalInput n = new DigitalInput(0);

  public Test_MagLimit() {}

  public boolean getData() {
    return n.get();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
