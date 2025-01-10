// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/*
 




  .______        ___       __          ___      .___  ___. 
  |   _  \      /   \     |  |        /   \     |   \/   | 
  |  |_)  |    /  ^  \    |  |       /  ^  \    |  \  /  | 
  |   _  <    /  /_\  \   |  |      /  /_\  \   |  |\/|  | 
  |  |_)  |  /  _____  \  |  `----./  _____  \  |  |  |  | 
  |______/  /__/     \__\ |_______/__/     \__\ |__|  |__| 
  




*/     

package frc.robot;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.Swerve.DriveSubsystem;

public class RobotContainer {

  // Drive Controller

  private XboxController m_controller = new XboxController(OIConstants.kDriveControllerPort);

  private JoystickButton xButton = new JoystickButton(m_controller, XboxController.Button.kX.value);
  private JoystickButton bButton = new JoystickButton(m_controller, XboxController.Button.kB.value);
  private JoystickButton aButton = new JoystickButton(m_controller, XboxController.Button.kA.value);

  // Subsystems

  private DriveSubsystem m_robotDrive = new DriveSubsystem();


  public RobotContainer() {
    configureBindings();    
    registedCommands();

    m_robotDrive.zeroHeading();



    m_robotDrive.setDefaultCommand(new RunCommand(
      () -> m_robotDrive.drive(
        -MathUtil.applyDeadband(m_controller.getLeftY(), OIConstants.kDriveDeadband), 
        -MathUtil.applyDeadband(m_controller.getLeftX(), OIConstants.kDriveDeadband),
        -MathUtil.applyDeadband(m_controller.getRightX(), OIConstants.kDriveDeadband), 
        false), 
      m_robotDrive));

  }

  private void configureBindings() {

    xButton.whileTrue(new RunCommand(() -> m_robotDrive.zeroHeading(), m_robotDrive));

    aButton.whileTrue(new RunCommand(() -> m_robotDrive.zeroPose(), m_robotDrive));
          
    bButton.onTrue(m_robotDrive.changeDriveModeCmd());

  } 

  private void registedCommands() {
  }

  public Command getAutonomousCommand() {
    return new Command() {
      
    };
  }
}
