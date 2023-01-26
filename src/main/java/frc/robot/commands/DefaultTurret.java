// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DefaultClawSystem;

public class DefaultTurret extends CommandBase {
  /** Creates a new DefaultTurret. */
  private XboxController primaryController;
  private XboxController secondaryController;
  private DefaultClawSystem defaultClawSystem;
  public DefaultTurret(DefaultClawSystem defaultClawSystem, XboxController primaryController, XboxController secondaryController) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(defaultClawSystem);
    this.primaryController = primaryController;
    this.secondaryController = secondaryController;
    this.defaultClawSystem = defaultClawSystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {


    //code for turntable
  if(primaryController.getRightBumper()){
    defaultClawSystem.spinTable(0.3);
  } else if(primaryController.getLeftBumper()){
    defaultClawSystem.spinTable(-0.3);
  } else {
    defaultClawSystem.spinTable(0);
  }

  //code for moving arm
  if(primaryController.getYButton()){
    defaultClawSystem.moveArm(0.2);
  } else if(primaryController.getXButton()){
    defaultClawSystem.moveArm(-0.2);
  } else{
    defaultClawSystem.moveArm(0);
  }

  //code for extending arm
  if(primaryController.getAButton()){
    defaultClawSystem.extendArm(0.2);
  } else if(primaryController.getBButton()){
    defaultClawSystem.extendArm(-0.2);
  } else{
    defaultClawSystem.extendArm(0);
  }

  if(secondaryController.getAButtonPressed()){
    defaultClawSystem.openClaw();
  } 
   if(secondaryController.getBButtonPressed()){
    defaultClawSystem.closeClaw();
  }


  if(secondaryController.getRightBumper()){ //extending arm on second controller
    defaultClawSystem.extendArm(.2);
  } else if (secondaryController.getLeftBumper()){
    defaultClawSystem.extendArm(-0.2);
  } else {
    defaultClawSystem.extendArm(0);
  }

defaultClawSystem.spinTable(secondaryController.getLeftX()/4);
defaultClawSystem.moveArm(secondaryController.getLeftY()/7*-1);

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
