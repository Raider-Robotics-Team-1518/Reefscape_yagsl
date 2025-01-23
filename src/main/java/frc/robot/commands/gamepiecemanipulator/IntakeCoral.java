// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.gamepiecemanipulator;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class IntakeCoral extends Command {
  private Timer timer;
  private boolean isDone = false;
  private double current_angle = RobotContainer.gpm.getCoralArmPosition();
  private double targetIntakeAngle = 30 * Constants.PI_OVER_180;

  public IntakeCoral(double targetIntakeAngle) {
    addRequirements(RobotContainer.gpm);
    this.targetIntakeAngle = targetIntakeAngle;
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
    double armRotationPower = Math.abs(this.targetIntakeAngle - current_angle) / 100;
    // Move arm up or down to target arm angle
    if (Math.abs(this.targetIntakeAngle - current_angle) > Constants.Tolerances.coralIntakeAngleTolerance) {
      double v_sign = Math.signum(this.targetIntakeAngle - current_angle);
      RobotContainer.gpm.setCoralArmSpeed(v_sign * (armRotationPower));
    } else {
      // arm is in the correct angle
      RobotContainer.gpm.setCoralArmSpeed(0d); // turn off arm motor
      RobotContainer.gpm.setCoralMotorSpeed(Constants.MotorSpeeds.coralIntakeMotorSpeed); // turn on intake motor
      if (RobotContainer.gpm.isCoralLoaded() || timer.hasElapsed(Constants.Times.coralIntakeMotorRunTime)) {
        RobotContainer.gpm.setCoralMotorSpeed(0d);
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
