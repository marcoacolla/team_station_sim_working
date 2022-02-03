// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class TurnRight extends CommandBase {
  /** Creates a new TurnRight. */
  private final DriveTrain driveTrain;
  private double axis = 10;
  private double turnLimit;
  private double lowError;
  private double highError;
  public TurnRight(DriveTrain dt, double turnValue) {
    // Use addRequirements() here to declare subsystem dependencies.
    driveTrain = dt;
    addRequirements(dt);



    //angulo para girar
    turnLimit = turnValue;
    lowError = turnLimit - 3;
    highError = turnLimit + 1;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    axis = driveTrain.gyro.getAngle();
    if(axis < lowError){
      driveTrain.arcadeAutonomousMove(0.0, 1.0, 0.5);
    }else if(axis > highError){
      driveTrain.arcadeAutonomousMove(0.0, -1.0, 0.5);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrain.gyro.reset();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(axis > lowError + 0.5 && axis < highError){
      return true;
    }else{
      return false;
    }
  }
}
