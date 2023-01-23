// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.SwerveJoystick;
import frc.robot.subsystems.SwerveSubsystem;

public class RobotContainer {

  public final static SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  public final static XboxController driverController = XboxController(OIConstants.kDriverControllerPort);

  public RobotContainer() {
    swerveSubsystem.setDefaultCommand(new SwerveJoystick(swerveSubsystem,
            () -> -driverController.getRawAxis(OIConstants.kDriverYAxis),
            () -> driverController.getRawAxis(OIConstants.kDriverXAxis),
            () -> driverController.getRawAxis(OIConstants.kDriverRotAxis),
            () -> !driverController.getAButton()));

    configureBindings();
  }

  
  private void configureBindings() {
  }
}