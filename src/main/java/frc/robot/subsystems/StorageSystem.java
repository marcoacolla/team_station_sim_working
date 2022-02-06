// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class StorageSystem extends SubsystemBase {
  /** Creates a new Storage. */
  private final VictorSP storageConv;
  public StorageSystem() {
    storageConv = new VictorSP(Constants.storageConvID);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void activateStorage(double speed){ 
      storageConv.set(speed);
   
  }
  public void stopStorage(){
    storageConv.set(0.0);
  }
  
}
