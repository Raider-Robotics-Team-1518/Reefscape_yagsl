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
  public static final double MAX_SPEED  = Units.feetToMeters(14.5);
  // Maximum speed of the robot in meters per second, used to limit acceleration.


  /* Factors of PI */
  public static final double PI_OVER_TWO = Math.PI * 0.5;
  public static final double THREE_PI_OVER_TWO = 3 * PI_OVER_TWO;
  public static final double TWO_PI = 2 * Math.PI;
  public static final Rotation2d ROTATE_BY_PI = Rotation2d.fromDegrees(180);
  public static final double PI_OVER_180 = Math.PI / 180;


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

  public static class Reef {
    public static final double[] levels = {
      0, // ignore this one
      0, // height of level 1 of the reef
      0, // level 2
      0, // level 3
      0  // level 4
    };
    public static final double armAngle1 = 10 * PI_OVER_180; // nearly horizontal
    public static final double armAngle23 = 30 * PI_OVER_180;
    public static final double armAngle4 = 90 * PI_OVER_180; // vertical
  }

  public static final class Tolerances {
    public static final double reefHeightTolerance = 1.0; // tolerance for height of the reef levels
    public static final double armAngleTolerance = 0.1; // angle of coral manipulator arm
  }

  public static final class MotorSpeeds {
    public static final double elevatorPower = 0.25;
    public static final double coralMotorSpeed = 0.5;
    public static final double algaeMotorSpeed = 0.5;
  }

  public static final class Times {
    public static final double coralMotorRunTime = 2.5;
    public static final double algaeMotorRunTime = 2.5;
  }
}
