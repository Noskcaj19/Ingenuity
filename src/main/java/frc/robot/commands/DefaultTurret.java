// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DefaultClawSystem;

public class DefaultTurret extends CommandBase {
  // Constant variables for speed
  private final double TableSpinSpeed = .3;
  private final double armMoveSpeed = .2;
  private double extendSet = 0;
  private final double armExtendSpeed = .6;
  private double set = 0;
  private double extendController = 0;
  private double moveController = 0;
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

  //code for turning the roller on and off :3
  if(secondaryController.getXButtonPressed()){
    defaultClawSystem.rollerIn();
  }
  if(secondaryController.getXButtonReleased()){
    defaultClawSystem.rollerStop();
  }
  if(secondaryController.getYButtonPressed()){
    defaultClawSystem.rollerOut();
  }
  if(secondaryController.getYButtonReleased()){
    defaultClawSystem.rollerStop();
  }
  

//extending arm on second controller
  // if(secondaryController.getRightBumper()){ 
  //   extendSet = MathUtil.clamp(extendSet + 0.5, 0, 45);
  //   defaultClawSystem.extendArm(extendSet);
  // } else if (secondaryController.getLeftBumper()){
  //   extendSet = MathUtil.clamp(extendSet - 0.5, 0, 45);
  //   defaultClawSystem.extendArm(extendSet);
  // } 
  // System.out.println(extendSet);
  // defaultClawSystem.extendArm(extendSet);


  // our own very special deadband method!!!
    if(secondaryController.getRightY() < 0.02 && secondaryController.getRightY() > -0.02){
      extendController = 0;
    } else {
      extendController = secondaryController.getRightY();
    }

    if(secondaryController.getLeftY() < 0.02 && secondaryController.getLeftY() > -0.02){
      moveController = 0;
    } else {
      moveController = secondaryController.getLeftY();
    }


  extendSet = MathUtil.clamp(-extendController + extendSet, 0, 45);
  defaultClawSystem.extendArm(extendSet);
  // else {
  //   defaultClawSystem.extendArm(extendSet);
  // }

  // //sets setpoint for PID
  // if(secondaryController.getXButton()){
  //   set = set + 0.1;
  // } 
  //  if(secondaryController.getYButton()){
  //   set = set - 0.1;
  // }
//secondarycontroller > 0.1 thing

  defaultClawSystem.spinTable(secondaryController.getLeftX()/8);
// defaultClawSystem.moveArm(secondaryController.getLeftX()*1/50);
//defaultClawSystem.moveArm(set);
set = (-moveController / 4) + set;
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
