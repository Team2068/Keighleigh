// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// https://static.garmin.com/pumac/LIDAR_Lite_v3_Operation_Manual_and_Technical_Specifications.pdf

public class LidarSubsystem extends SubsystemBase {

  public enum LidarConfiguration {
    DEFAULT, // Default mode, balanced performance
    SHORT_RANGE, // Short range, high speed
    DEFAULT_HIGH_SPEED, // Default range, higher speed short range
    MAXIMUM_RANGE, // Maximum range
    HIGH_SENSITIVE, // High sensitivity detection, high erroneous measurements
    LOW_SENSITIVE // Low sensitivity detection, low erroneous measurements
  }

  private I2C _lidar;

  private LidarConfiguration config = null;

  public LidarSubsystem() {
    this(LidarConfiguration.DEFAULT, (byte) 0x62);
  }

  public LidarSubsystem(LidarConfiguration configuration, int address) {
    _lidar = new I2C(Port.kMXP, 0x62); // default i2c port

    // if the address isn't 0x62 (default address) change the address
    if (address != 0x62) {
      setI2cAddress((byte) address);
    }

    changeMode(configuration);
  }

  private void setI2cAddress(byte newAddress) {
    // read and write back serial number
    // to ensure that we have the correct sensor
    byte[] dataBytes = new byte[2];
    _lidar.read(0x8F, 2, dataBytes);
    _lidar.write(0x18, dataBytes[0]);
    _lidar.write(0x19, dataBytes[1]);

    // Write the new I2C device address to registers
    dataBytes[0] = newAddress;
    _lidar.write(0x1a, dataBytes[0]);

    // Enable the new I2C device address using the default I2C device address
    dataBytes[0] = 0;
    _lidar.write(0x1e, dataBytes[0]);

    // delete old object, create new one at new address
    _lidar = null;
    _lidar = new I2C(Port.kMXP, newAddress);

    // disable default I2C device address (using the new I2C device address)
    dataBytes[0] = (1 << 3); // set bit to disable default address
    _lidar.write(0x1e, dataBytes[0]);
  }

  public void changeMode(LidarConfiguration configuration) {
    switch (configuration) {
      case DEFAULT: // Default mode, balanced performance
        _lidar.write(0x02, 0x80); // Default
        _lidar.write(0x04, 0x08); // Default
        _lidar.write(0x1c, 0x00); // Default
        break;

      case SHORT_RANGE: // Short range, high speed
        _lidar.write(0x02, 0x1d);
        _lidar.write(0x04, 0x08); // Default
        _lidar.write(0x1c, 0x00); // Default
        break;

      case DEFAULT_HIGH_SPEED: // Default range, higher speed short range
        _lidar.write(0x02, 0x80); // Default
        _lidar.write(0x04, 0x00);
        _lidar.write(0x1c, 0x00); // Default
        break;

      case MAXIMUM_RANGE: // Maximum range
        _lidar.write(0x02, 0xff);
        _lidar.write(0x04, 0x08); // Default
        _lidar.write(0x1c, 0x00); // Default
        break;

      case HIGH_SENSITIVE: // High sensitivity detection, high erroneous measurements
        _lidar.write(0x02, 0x80); // Default
        _lidar.write(0x04, 0x08); // Default
        _lidar.write(0x1c, 0x80);
        break;

      case LOW_SENSITIVE: // Low sensitivity detection, low erroneous measurements
        _lidar.write(0x02, 0x80); // Default
        _lidar.write(0x04, 0x08); // Default
        _lidar.write(0x1c, 0xb0);
        break;
    }
    config = configuration;
  }

  public double getDistance(boolean biasCorrection) {
    if (biasCorrection)
      _lidar.write(0x00, 0x04);
    else
      _lidar.write(0x00, 0x03);

    byte[] distanceArray = new byte[2];

    _lidar.read(0x8F, 2, distanceArray);

    double distance = (distanceArray[0] << 8) + distanceArray[1];

    return distance;
  }

  public void reset() {
    _lidar.write(0x00, 0x00);
  }

  public LidarConfiguration getCurrentConfiguration() {
    return config;
  }

  @Override
  public void periodic() {
    double distance = getDistance(true) / 2.54; // converting to inches here :D
    SmartDashboard.putNumber("Lidar Sensor Distance", distance);
    String currentConfig = "";

    switch (getCurrentConfiguration()) {
      case DEFAULT:
        currentConfig = "Default";
        break;
      case SHORT_RANGE:
        currentConfig = "Short Range";
        break;
      case DEFAULT_HIGH_SPEED:
        currentConfig = "Default High Speed";
        break;
      case MAXIMUM_RANGE:
        currentConfig = "Maxmimum Range";
        break;
      case HIGH_SENSITIVE:
        currentConfig = "High sensitivity";
        break;
      case LOW_SENSITIVE:
        currentConfig = "Low sensitivity";
        break;
    }
    SmartDashboard.putString("Lidar Config", currentConfig);
  }
}
