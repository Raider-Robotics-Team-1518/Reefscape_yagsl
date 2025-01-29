// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
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
  // probably not going to all be TalonFXs
  private TalonFX elevatorMotor;
  private TalonFX wristMotor;
  private TalonFX coralMotor;
  private double elevatorPosition = 0;
  private double coralArmPosition = 0;

  
  public GamePieceManipulator() {
    elevatorMotor = new TalonFX(Constants.Motors.elevatorMotorId);
    wristMotor = new TalonFX(Constants.Motors.wristMotorId);
    coralMotor = new TalonFX(Constants.Motors.coralMotorId);
    // TODO: reset the encoders to 0 here
  }

  public void driveElevator(double speed) {
    // TODO: this is probably not correct even if we do use a TalonFX
    elevatorPosition = getCurrentHeight();
    if (elevatorPosition > Constants.Limits.elevatorMin && elevatorPosition < Constants.Limits.elevatorMax) {
      elevatorMotor.set(speed);
    } else {
      elevatorMotor.set(0.0);
    }
  }

  public void stopElevator() {
    elevatorMotor.set(0d);
  }

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

  public void setCoralMotorSpeed(double speed) {
    // positive for ejecting, negative for intaking
    coralMotor.set(speed);
  }

  public void setWristSpeed(double speed) {
    // positive for rotating towards a more vertical angle
    double currentWristPosition = getWristPosition();
    if (currentWristPosition > Constants.Limits.wristMinAngle && currentWristPosition < Constants.Limits.wristMaxAngle) {
      wristMotor.set(speed);
    } else {
      wristMotor.set(0);
    }
  }

  public double getCurrentHeight() {
    // get current height of the elevator
    // TODO: this is probably not correct even if we do use a TalonFX
    elevatorPosition = elevatorMotor.getPosition().getValueAsDouble(); // this is in rotations
    return elevatorPosition * Constants.Factors.elevatorInchesPerRevolution;

  }
  public double getWristPosition() {
    // get the angle of the coral arm/wrist/whatever we're going to call it
    // TODO: this is probably not correct even if we do use a TalonFX
    coralArmPosition = wristMotor.getPosition().getValueAsDouble(); // rotations
    return coralArmPosition * Constants.Factors.wristDegreesPerRevolution; // degrees, but should it be radians?
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
