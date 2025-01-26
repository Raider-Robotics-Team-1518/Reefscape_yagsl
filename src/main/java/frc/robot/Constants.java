// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.hardware.TalonFX;

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

  public static class Motors {
    public static final int elevatorMotorId = 0;
    public static final int algaeArmMotorId = 1;
    public static final int coralArmMotorId = 2;
    public static final int coralMotorId = 3;
    public static final int algaeMotorId = 4;
  }

  public static class Limits {
    public static final double elevatorMax = 100;
    public static final double elevatorMin = 0;
    public static final double wristMinAngle = 0;
    public static final double wristMaxAngle = 180;
  }

  public static class Factors {
    public static final double elevatorInchesPerRevolution = 1;
    public static final double coralArmDegreesPerRevolution = 360;
    public static final double algaeArmDegreesPerRevolution = 360;
  }

  public static class Reef {
    public static final double[] levels = {
      0, // ignore this one
      0, // height of level 1 of the reef
      0, // level 2
      0, // level 3
      0  // level 4
    };
    // optimal angle to eject coral onto level 1
    public static final double coralEjectAngleLevel1 = 10 * PI_OVER_180; // nearly horizontal
    public static final double coralEjectAngleLevel23 = 30 * PI_OVER_180;
    public static final double coralEjectAngleLevel4 = 90 * PI_OVER_180; // vertical
    // optimal angle for ejecting algae into the processor
    public static final double algaeEjectArmAngle = 20 * PI_OVER_180;
    public static final double targetCoralIntakeAngle = 70 * PI_OVER_180;
    public static final double targetAlgaeIntakeAngle = 20 * PI_OVER_180;
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
    public static final double coralIntakeMotorRunTime = 4;
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
