// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Utils;


public class GamePieceManipulator extends SubsystemBase {
  private final I2C.Port i2cPort = I2C.Port.kMXP;
  private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);

  
  public GamePieceManipulator() {}

  public void driveElevator(double power) {}

  public void stopElevator() {}

  public boolean isCoralLoaded() {
    // read the color/presence sensor to see if the coral has been loaded
    Color detectedColor = m_colorSensor.getColor(); // returns a struct of doubles
    double r = detectedColor.red;
    double b = detectedColor.blue;
    double g = detectedColor.green;

    // calculate with Hue Saturation Value (HSV), may need
    // to consider the saturation in addition to value
    float value = Utils.getValue((float)r, (float)g, (float)b);

    if (value > Constants.ColorValues.whiteValueMin) {
      SmartDashboard.putBoolean("Coral Loaded", true);
      return true;
    }
    else {
      SmartDashboard.putBoolean("Coral Loaded", false);
      return false;
    }
  }

  public void setAlgaeMotorSpeed(double speed) {
    // positive for ejecting, negative for intaking
  }

  public void setAlgaeArmSpeed(double speed) {
    // positive for rotating towards a more vertical angle
  }

  public void setCoralMotorSpeed(double speed) {
    // positive for ejecting, negative for intaking
  }

  public void setCoralArmSpeed(double speed) {
    // positive for rotating towards a more vertical angle
  }

  public double getCurrentHeight() {
    // get current height of the elevator
    return 0d;
  }
  public double getCoralArmPosition() {
    // get the angle of the coral arm/wrist/whatever we're going to call it
    return 0d;
  }

  public double getAlgaeArmPosition() {
    // get the angle of the algae arm/wrist/whatever we're going to call it
    return 0d;
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
