// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/*
 * Class to eject the coral onto the reef
 */

package frc.robot.commands.gamepiecemanipulator;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class EjectCoral extends Command {
  private Timer timer;
  private boolean isDone = false;
  private double current_angle = RobotContainer.gpm.getCoralArmPosition();
  private double targetArmAngle = 30 * Constants.PI_OVER_180;

  public EjectCoral(double targetArmAngle) {
    addRequirements(RobotContainer.gpm);
    this.targetArmAngle = targetArmAngle; // will be one of the Constants.Reef.armAngle values
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer = new Timer();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    timer.start();
    // set arm to correct angle
    current_angle = RobotContainer.gpm.getCoralArmPosition();
    // Calculate power curve proportional
    double armRotationPower = Math.abs(this.targetArmAngle - current_angle) / 100;
    // Move arm up or down to target arm angle
    if (Math.abs(this.targetArmAngle - current_angle) > Constants.Tolerances.coralEjectAngleTolerance) {
      double v_sign = Math.signum(this.targetArmAngle - current_angle);
      RobotContainer.gpm.setCoralArmSpeed(v_sign * (armRotationPower));
    } else {
      RobotContainer.gpm.setCoralArmSpeed(0d);
      RobotContainer.gpm.setCoralMotorSpeed(Constants.MotorSpeeds.coralEjectMotorSpeed);
      if (timer.hasElapsed(Constants.Times.coralEjectMotorRunTime)) {
        isDone = true;
        isFinished();
      }
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    timer.stop();
    RobotContainer.gpm.setCoralMotorSpeed(0);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isDone;
  }
}
