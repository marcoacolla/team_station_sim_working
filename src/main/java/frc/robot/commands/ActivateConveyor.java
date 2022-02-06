// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.StorageSystem;

public class ActivateConveyor extends CommandBase {
  private final StorageSystem storageSystem;
  private double convSpeed;
  /** Creates a new activateConveyor. */
  public ActivateConveyor(StorageSystem st, double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    convSpeed = speed;
    storageSystem = st;
    addRequirements(st);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    storageSystem.activateStorage(convSpeed);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    storageSystem.stopStorage();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
