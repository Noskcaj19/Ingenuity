// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class DefaultDrive extends CommandBase {
  private XboxController primaryController;
  private Drivetrain drivetrain;
  // pretend that there is a second controller
  
  /** Creates a new DefaultDrive. */
  public DefaultDrive(Drivetrain drivetrain, XboxController primaryController) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
    this.primaryController = primaryController;
    this.drivetrain = drivetrain;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    
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
