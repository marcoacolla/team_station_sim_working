// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.ActivateConveyor;
//import frc.robot.commands.InitialPosition;
import frc.robot.commands.TurnRight;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.StorageSystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
//import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.driveWithJoysticks;
import frc.robot.commands.invertedDriveWithJoysticks;
//import frc.robot.commands.moveForward;
/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  //declarando DriveTrain:
  private final DriveTrain driveTrain;
  private final StorageSystem storageSystem;
  private final driveWithJoysticks driveWithJoysticks;
  //private final ActivateConveyor activateConveyor;
  //private final InitialPosition initialPosition;
  private static XboxController controller;
  private final JoystickButton invertButton;
  private final JoystickButton convTrigger;

  
  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    driveTrain = new DriveTrain();
    storageSystem = new StorageSystem();
    controller = new XboxController(Constants.joytickID);
    driveWithJoysticks = new driveWithJoysticks(driveTrain, controller);
    //activateConveyor = new ActivateConveyor(storageSystem);
    //initialPosition = new InitialPosition(driveTrain);
    driveTrain.setDefaultCommand(driveWithJoysticks);    
    invertButton = new JoystickButton(controller, XboxController.Button.kA.value);
    convTrigger =  new JoystickButton(controller, XboxController.Button.kStart.value);

    // Configure the button bindings
    configureButtonBindings();

  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    convTrigger.whenHeld(new ActivateConveyor(storageSystem, 1.0));
    invertButton.whenHeld(new invertedDriveWithJoysticks(driveTrain, controller));
  }
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    driveTrain.reset();
    return new SequentialCommandGroup(
      new TurnRight(driveTrain, 60)
    );
  
  }
  
}
