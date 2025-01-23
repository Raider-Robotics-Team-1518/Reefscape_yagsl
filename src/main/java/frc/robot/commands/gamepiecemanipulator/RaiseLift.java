// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/*
 * Class to handle raising the lift to a specified level of the reef
 */

package frc.robot.commands.gamepiecemanipulator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class RaiseLift extends Command {
  private int level = 0;
  private double currentHeight = 0;
  private double targetHeight = 0;
  private boolean isDone = false;
  
  public RaiseLift(int level) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.gpm);
    // level will be one of 1, 2, 3, or 4
    this.level = level;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currentHeight = RobotContainer.gpm.getCurrentHeight();
    targetHeight = Constants.Reef.levels[this.level];    
    if (Math.abs(targetHeight - currentHeight) > Constants.Tolerances.reefHeightTolerance) {
      double v_sign = Math.signum(targetHeight - currentHeight);
      RobotContainer.gpm.driveElevator(v_sign * (Constants.MotorSpeeds.elevatorPower));
    } else {
      RobotContainer.gpm.stopElevator();
      isDone = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isDone;
  }
}
