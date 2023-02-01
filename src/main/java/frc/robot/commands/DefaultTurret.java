// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DefaultClawSystem;

public class DefaultTurret extends CommandBase {
  // Constant variables for speed
  private final double TableSpinSpeed = .3;
  private final double armMoveSpeed = .2;
  private final double armExtendSpeed = .6;
  private double set = 0;
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
    defaultClawSystem.spinTable(TableSpinSpeed);
  } else if(primaryController.getLeftBumper()){
    defaultClawSystem.spinTable(-TableSpinSpeed);
  } else {
    defaultClawSystem.spinTable(0);
  }

  

  //code for extending arm
  if(primaryController.getAButton()){
    defaultClawSystem.extendArm(armExtendSpeed);
  } else if(primaryController.getBButton()){
    defaultClawSystem.extendArm(-armExtendSpeed);
  } else{
    defaultClawSystem.extendArm(0);
  }

  if(secondaryController.getAButtonPressed()){
    defaultClawSystem.openClaw();
  } 
   if(secondaryController.getBButtonPressed()){
    defaultClawSystem.closeClaw();
  }

//extending arm on second controller
  if(secondaryController.getRightBumper()){ 
    defaultClawSystem.extendArm(armExtendSpeed);
  } else if (secondaryController.getLeftBumper()){
    defaultClawSystem.extendArm(-armExtendSpeed);
  } else {
    defaultClawSystem.extendArm(0);
  }

  //sets setpoint for PID
  if(secondaryController.getXButton()){
    set = set + 0.1;
  } 
   if(secondaryController.getYButton()){
    set = set - 0.1;
  }

defaultClawSystem.spinTable(secondaryController.getLeftX()/4);
// defaultClawSystem.moveArm(secondaryController.getLeftX()*1/50);
defaultClawSystem.moveArm(set);


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
