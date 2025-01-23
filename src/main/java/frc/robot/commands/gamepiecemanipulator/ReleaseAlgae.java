// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/*
 * Class to eject the algae into the processor
 * 
 * NEEDS TO BE UPDATED TO SET THE ANGLE OF THE MECHANISM BEFORE RELEASING THE BALL
 */

 package frc.robot.commands.gamepiecemanipulator;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ReleaseAlgae extends Command {
  private Timer timer;
  private boolean isDone = false;

  public ReleaseAlgae() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.gpm);
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
    RobotContainer.gpm.setAlgaeMotorSpeed(Constants.MotorSpeeds.algaeMotorSpeed);
    if (timer.hasElapsed(Constants.Times.algaeMotorRunTime)) {
      isDone = true;
      isFinished();
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    timer.stop();
    RobotContainer.gpm.setAlgaeMotorSpeed(0);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isDone;
  }
}
