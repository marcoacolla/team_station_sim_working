// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveTrain;

public class driveWithJoysticks extends CommandBase {
  /** Creates a new driveWithJoysticks. */
  private final DriveTrain driveTrain;
  private double invertTrigger;

  public driveWithJoysticks(DriveTrain dt) {
    // Use addRequirements() here to declare subsystem dependencies.
    driveTrain = dt;
    addRequirements(dt);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    invertTrigger = RobotContainer.stick.getRawAxis(Constants.invertTriggerID);
  
    if (invertTrigger == 1.0) {
      driveTrain.invertedArcadeJoysticks(RobotContainer.stick,Constants.robot_speed);

    }else{ 
      driveTrain.arcadeJoysticks(RobotContainer.stick, Constants.robot_speed);
    }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
