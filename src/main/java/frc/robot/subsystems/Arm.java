// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxRelativeEncoder.Type;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ELBOW;
import frc.robot.Constants.SHOULDER;
import frc.robot.Constants.WRIST;

public class Arm extends SubsystemBase {
  /** Creates a new Arm. */
  public static final CANSparkMax leftShoulder = new CANSparkMax(SHOULDER.LEFT, MotorType.kBrushless);
  public static final CANSparkMax rightShoulder = new CANSparkMax(SHOULDER.RIGHT, MotorType.kBrushless);
  public static final CANSparkMax elbow = new CANSparkMax(ELBOW.ID, MotorType.kBrushless);
  public static final CANSparkMax wrist = new CANSparkMax(WRIST.ID, MotorType.kBrushless);

  public static final Solenoid airClaw = new Solenoid(21, PneumaticsModuleType.REVPH, 8);

  public static final RelativeEncoder shoulderEncoder = leftShoulder.getEncoder(Type.kHallSensor , 42);
  public static final RelativeEncoder elbowEncoder = elbow.getEncoder(Type.kHallSensor , 42);
  public static final RelativeEncoder wristEncoder = wrist.getEncoder(Type.kHallSensor , 42);

  public static final PIDController shoulderPID = new PIDController(SHOULDER.kP, SHOULDER.kI, SHOULDER.kD);
  public static final PIDController elbowPID = new PIDController(ELBOW.kP, ELBOW.kI, ELBOW.kD);
  public static final PIDController wristPID = new PIDController(WRIST.kP, WRIST.kI, WRIST.kD);

  public Arm() {
    leftShoulder.restoreFactoryDefaults();
    rightShoulder.restoreFactoryDefaults();
    wrist.restoreFactoryDefaults();
    elbow.restoreFactoryDefaults();
    elbow.setIdleMode(IdleMode.kBrake);
    leftShoulder.setIdleMode(IdleMode.kBrake);
    rightShoulder.setIdleMode(IdleMode.kBrake);
    wrist.setIdleMode(IdleMode.kBrake);
    elbowEncoder.setPosition(0);
    shoulderEncoder.setPosition(0);
    wristEncoder.setPosition(0);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Elbow Position", elbowEncoder.getPosition());
    SmartDashboard.putNumber("Shoulder Position", shoulderEncoder.getPosition());
    SmartDashboard.putNumber("Elbow Temp", elbow.getMotorTemperature());
    SmartDashboard.putNumber("Wrist Position", wristEncoder.getPosition());
    SmartDashboard.putNumber("Wrist Temp", wrist.getMotorTemperature());
  }

  public void shoulderPIDPosition(double position){
    moveShoulder(shoulderPID.calculate(shoulderEncoder.getPosition(), position));
  }

  public void elbowPIDPosition(double position){
    moveElbow(elbowPID.calculate(elbowEncoder.getPosition(), position));
  }

  public void wristPIDPosition(double position){
    moveWrist(wristPID.calculate(wristEncoder.getPosition(), position));
  }

  public void airClaw(boolean state){
      airClaw.set(state);
  }

  public void elbowToPosition(double position){
    if(elbowEncoder.getPosition() > position){
      moveElbow(-0.05);
    }else if(elbowEncoder.getPosition() < position){
      moveElbow(ELBOW.MAX_SPEED);
    }else{
      moveElbow(0);
    }
  }

  public void wristToPosition(double position){
    if(wristEncoder.getPosition() > position){
      moveWrist(-0.05);
    }else if(wristEncoder.getPosition() < position){
      moveWrist(WRIST.MAX_SPEED);
    }else{
      moveWrist(0);
    }
  }

  public void resetPositionAll(){
    elbowEncoder.setPosition(0);
    shoulderEncoder.setPosition(0);
    wristEncoder.setPosition(0);
  }
  
  public void moveShoulder(double speed){
    leftShoulder.set(speed);
    rightShoulder.set(-speed);
  }

  public void moveElbow(double speed){
    elbow.set(speed);
  }

  public void moveWrist(double speed){
    wrist.set(speed);
  }
}
