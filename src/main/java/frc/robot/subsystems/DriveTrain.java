// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.simulation.ADXRS450_GyroSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotGearing;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotMotor;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotWheelSize;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveTrain extends SubsystemBase {
  /** Creates a new DriveTrain. */

  //gyro
  public ADXRS450_Gyro gyro = new ADXRS450_Gyro();
  //simulação do gyro
  public ADXRS450_GyroSim gyroSim = new ADXRS450_GyroSim(gyro);
  //declarando motores left
  public WPI_TalonSRX leftMaster = new WPI_TalonSRX(Constants.leftMasterID);
  public WPI_TalonSRX leftSlave = new WPI_TalonSRX(Constants.leftSlaveID);

  //declarando motores right
  public WPI_TalonSRX rightMaster = new WPI_TalonSRX(Constants.rightMasterID);
  public WPI_TalonSRX rightSlave = new WPI_TalonSRX(Constants.rightSlaveID);
  //diferential Drive
  DifferentialDrive m_drive;
  //speedcontrollers
  SpeedController leftMotor;
  SpeedController rightMotor;

    //driveTrain sim
    private DifferentialDrivetrainSim driveSim = DifferentialDrivetrainSim.createKitbotSim(
      KitbotMotor.kDualCIMPerSide,
      KitbotGearing.k10p71,
      KitbotWheelSize.SixInch,
      null
    );
    private Field2d field = new Field2d();
    private final DifferentialDriveOdometry odometry;

  public DriveTrain() {

    //definindo speedcontrollers
    leftSlave.follow(leftMaster);
    rightSlave.follow(rightMaster);

    leftMaster.setInverted(false);
    rightMaster.setInverted(true);
    leftSlave.setInverted(InvertType.FollowMaster);
    rightSlave.setInverted(InvertType.FollowMaster);
    //differentialDrive
    m_drive = new DifferentialDrive(leftMaster, rightMaster);

    //Sim
    SmartDashboard.putData("Field", field);
    odometry = new DifferentialDriveOdometry(driveSim.getHeading());

  }
 @Override
  public void periodic() {
    // This method will be called once per scheduler run
    odometry.update(driveSim.getHeading(), driveSim.getLeftPositionMeters(), driveSim.getLeftPositionMeters());
    field.setRobotPose(odometry.getPoseMeters());
  }

  public void invertedArcadeJoysticks(XboxController stick, double speed){

    m_drive.arcadeDrive(stick.getRawAxis(Constants.yID)*speed, stick.getRawAxis(Constants.xID)*speed);
    }

  public void arcadeJoysticks(XboxController stick, double speed){

  m_drive.arcadeDrive(-stick.getRawAxis(Constants.yID)*speed, stick.getRawAxis(Constants.xID)*speed);
  }

  public void arcadeAutonomousMove(double moveSpeed, double turnSpeed, double speed){
    m_drive.arcadeDrive(moveSpeed*speed, turnSpeed*speed);
  }

  public void simulationPeriodic(){
    driveSim.setInputs(
      leftMaster.get() * RobotController.getInputVoltage(),
      -rightMaster.get() * RobotController.getInputVoltage()
    );
    driveSim.update(0.02);

    gyroSim.setAngle(-driveSim.getHeading().getDegrees());
  }

  public void stop(){
    m_drive.stopMotor();
  }
  public void reset(){
    odometry.resetPosition(new Pose2d(), new Rotation2d());
    driveSim.setPose(odometry.getPoseMeters());
  }


}



