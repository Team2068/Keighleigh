// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.SerialPort.StopBits;

public class Test_Sonic {
  SerialPort RS_port = new SerialPort(9600, Port.kOnboard, 8, SerialPort.Parity.kNone, StopBits.kOne);
  

  public Test_Sonic() {}

  public int getDistance(){
    byte[] vals = RS_port.read(5); // First Byte is R, 4 data and the last is ASCII 13
    // assert(vals[0] == '5'); // For Testing
    if (vals[0] != 'R' || vals[4] != 13)
      DriverStation.reportWarning("Val returned by MB1040 possibly corrupt", false);
    System.out.println(vals);
    return vals[1];    
  }
}
