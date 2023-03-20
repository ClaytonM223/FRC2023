// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.ELBOW;
import frc.robot.Constants.SHOULDER;
import frc.robot.Constants.USB;
import frc.robot.Constants.WRIST;

public class ArmControl extends CommandBase {
  /** Creates a new ArmControl. */
  public ArmControl() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.arm.moveShoulder(0);
    RobotContainer.arm.moveElbow(0);
    RobotContainer.arm.moveWrist(0);
    //RobotContainer.arm.moveClaw(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    RobotContainer.arm.moveShoulder(RobotContainer.operatorController.getRawAxis(USB.OPERATOR_LY) * SHOULDER.MAX_SPEED);
    RobotContainer.arm.moveWrist(-(RobotContainer.operatorController.getRawAxis(USB.OPERATOR_RT) * WRIST.MAX_SPEED));
    RobotContainer.arm.moveWrist(RobotContainer.operatorController.getRawAxis(USB.OPERATOR_LT) * -WRIST.MAX_SPEED);
    RobotContainer.arm.moveElbow(RobotContainer.operatorController.getRawAxis(USB.OPERATOR_RY) * ELBOW.MAX_SPEED);

    if(RobotContainer.operatorController.getRawAxis(USB.OPERATOR_LT) > 0.05){
      RobotContainer.arm.moveWrist(RobotContainer.operatorController.getRawAxis(USB.OPERATOR_LT) * WRIST.MAX_SPEED);
    }else if(RobotContainer.operatorController.getRawAxis(USB.OPERATOR_RT) > 0.05){
      RobotContainer.arm.moveWrist(RobotContainer.operatorController.getRawAxis(USB.OPERATOR_RT) * -WRIST.MAX_SPEED);
    }

    if(RobotContainer.operatorController.getStartButton()){
      RobotContainer.arm.resetPositionAll();
    }

    if(RobotContainer.operatorController.getRightBumper()){
      RobotContainer.arm.airClaw(true);
      //RobotContainer.arm.clawPIDPosition(CLAW.OPEN_POSITION);
    }
    
    if(RobotContainer.operatorController.getLeftBumper()){
      RobotContainer.arm.airClaw(false);
      //RobotContainer.arm.clawPIDPosition(CLAW.CLOSED_POSITION);
    }

    if(RobotContainer.operatorController.getXButtonPressed()){
      RobotContainer.arm.moveShoulder(-0.2);
    }

    if(RobotContainer.operatorController.getXButtonReleased()){
      RobotContainer.arm.moveShoulder(0);
    }

    if(RobotContainer.operatorController.getYButtonPressed()){
      RobotContainer.arm.moveShoulder(0.05);
    }

    if(RobotContainer.operatorController.getYButtonReleased()){
      RobotContainer.arm.moveShoulder(0);
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
