// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.ARM;

public class Arm extends SubsystemBase {
  /** Creates a new Arm. */
  public static final CANSparkMax leftShoulder = new CANSparkMax(ARM.LEFT_SHOULDER, MotorType.kBrushless);
  public static final CANSparkMax rightShoulder = new CANSparkMax(ARM.RIGHT_SHOULDER, MotorType.kBrushless);
  public static final CANSparkMax elbow = new CANSparkMax(ARM.ELBOW, MotorType.kBrushless);

  public Arm() {
    leftShoulder.restoreFactoryDefaults();
    rightShoulder.restoreFactoryDefaults();
    elbow.restoreFactoryDefaults();
    elbow.setIdleMode(IdleMode.kBrake);
    leftShoulder.setIdleMode(IdleMode.kBrake);
    rightShoulder.setIdleMode(IdleMode.kBrake);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  
  public void moveShoulder(double speed){
    leftShoulder.set(speed);
    rightShoulder.set(-speed);
  }

  public void moveElbow(double speed){
    elbow.set(speed);
  }
}
