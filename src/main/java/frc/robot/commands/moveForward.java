// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class moveForward extends CommandBase {
  /** Creates a new moveForward. */
  private final DriveTrain driveTrain;
  private Timer timer;
  private double timeLimit;
  private double timerD;

  public moveForward(DriveTrain dt, double tL) {
    // Use addRequirements() here to declare subsystem dependencies
    driveTrain = dt;
    addRequirements(dt);
    // função timer
    timeLimit = tL;
    timer = new Timer();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    timerD = timer.get();

    driveTrain.arcadeAutonomousMove(1.0, 0.0, 0.5);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrain.arcadeAutonomousMove(0.0, 0.0, 0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timerD > timeLimit;
  }
}
