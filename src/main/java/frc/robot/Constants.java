// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants
{

  public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag
  // Maximum speed of the robot in meters per second, used to limit acceleration.
  public static final double MAX_SPEED  = Units.feetToMeters(14.5);


//  public static final class AutonConstants
//  {
//
//    public static final PIDConstants TRANSLATION_PID = new PIDConstants(0.7, 0, 0);
//    public static final PIDConstants ANGLE_PID       = new PIDConstants(0.4, 0, 0.01);
//  }

  public static final class DrivebaseConstants
  {

    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }

  public static class OperatorConstants
  {

    // Joystick Deadband
    public static final double DEADBAND        = 0.1;
    public static final double LEFT_Y_DEADBAND = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT    = 6;
  }

  public static class Motors {
    public static final int elevatorMotorId = 16;
    public static final int wristMotorId = 17;
    public static final int coralMotorId = 18;
  }

  public static class Limits {
    public static final double elevatorMax = 100;
    public static final double elevatorMin = 0;
    public static final double wristMinAngle = 0;
    public static final double wristMaxAngle = 180;
  }

  public static class Factors {
    public static final double elevatorInchesPerRevolution = 1;
    public static final double wristDegreesPerRevolution = 360;
  }

  public static class Reef {
    // Per Q&A system: "Keep in mind that dimensions on the REEF
    // and other structures have a tolerance of +/- 1/2 in. to
    // accommodate variances in manufacturing and assembly."
    public static final double[] levels = {
      0,      // we ignore this level, leave at 0
      18,     // height in inches of level 1 of the reef
      31.875, // level 2
      47.625, // level 3
      72      // level 4
    };
    // optimal angle in degrees to eject coral onto level 1
    public static final double coralEjectAngleLevel1 = 10; // nearly horizontal
    public static final double coralEjectAngleLevel23 = 30;
    public static final double coralEjectAngleLevel4 = 90; // vertical
    // optimal angle in degrees for ejecting algae into the processor
    public static final double algaeEjectArmAngle = 20;
    public static final double targetCoralIntakeAngle = 70;
    public static final double targetAlgaeIntakeAngle = 20;
  }

  public static final class Tolerances {
    public static final double reefHeightTolerance = 1.0; // tolerance for height of the reef levels
    public static final double coralEjectAngleTolerance = 0.1; // angle of coral manipulator arm
    public static final double coralIntakeAngleTolerance = 0.1;
    public static final double algaeEjectAngleTolerance = 0.1;
    public static final double algaeIntakeAngleTolerance = 0.1;
    public static final double minElevatorHeightTolerance = 0.1;
  }

  public static final class MotorSpeeds {
    public static final double elevatorPower = 0.25;
    // TODO: either intake or eject need to have negative values
    public static final double coralEjectMotorSpeed = 0.5;
    public static final double coralIntakeMotorSpeed = 0.25;
    public static final double algaeEjectMotorSpeed = 0.5;
    public static final double algaeIntakeMotorSpeed = 0.5;
  }

  public static final class Times {
    public static final double coralEjectMotorRunTime = 2.5;
    public static final double coralIntakeMotorRunTime = 10;
    public static final double algaeMotorRunTime = 2.5;
  }

  public static final class ColorValues {
     // 95% value would be a very light version of any color
    public static final float whiteValueMin = 95f;
    // For HSL, define the min & max of HUE that is the target color
    // see https://hslpicker.com/#ff6a00
    public static final float whiteHueMin = 30.0f;
    public static final float whiteHueMax = 80.0f;

  }
}
