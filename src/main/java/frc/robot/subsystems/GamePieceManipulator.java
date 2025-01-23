// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class GamePieceManipulator extends SubsystemBase {
  /** Creates a new GamePieceManipulator. */
  public GamePieceManipulator() {}

  public void driveElevator(double power) {}

  public void stopElevator() {}

  public void setAlgaeMotorSpeed(double speed) {
    // positive for ejecting, negative for intaking
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
  public double getArmPosition() {
    // get the angle of the coral arm/wrist/whatever we're going to call it
    return 0d;
  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
