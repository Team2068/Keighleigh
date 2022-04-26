// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableInstance;

public class RobotState {
  static NetworkTableInstance mainTable = NetworkTableInstance.getDefault();

  public static <T> void setEntryValue(String Table, String Entry, T val){
    mainTable.getTable(Table).getEntry(Entry).setValue(val);
  }

  public static <T> T getEntryValue(String Table, String Entry){
    return (T) mainTable.getTable(Table).getEntry(Entry).getValue();
  }
}
