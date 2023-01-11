// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

    private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();

    private final Joystick driverJoystick = new Joystick(OIConstants,kDriverControllerPort);


    public RobotContainer() {
        swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
          () -> -driverJoystick.getRawAxis(OIConstants.kDriverYAxis),
          () -> driverJoystick.getRawAxis(OIConstants.kDriveXAxis),
          () -> driverJoystick.getRawAxis(OIConstants.kDriverRotAxis),
          () -> !driverJoystick.getRawAxis(OIConstants.kDriverFieldOrientedButtonIdx)));
      configureButtonBindings();
    }

    private void configureButtonBindings() {
    }

    public Command getAutonomousCommand() {
      return null;
    }

  
 
}
