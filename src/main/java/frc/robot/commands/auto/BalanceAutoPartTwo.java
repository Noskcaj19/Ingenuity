// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import java.lang.reflect.InaccessibleObjectException;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class BalanceAutoPartTwo extends CommandBase {
  // are we at the angle that we should go to the next command?
  private Drivetrain drivetrain;
  private double speed;

  /** Creates a new BalanceAuto. */
  public BalanceAutoPartTwo(Drivetrain drivetrain, double speed) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.drivetrain = drivetrain;
    this.speed = speed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // System.out.println(navx.getYaw());
    if (drivetrain.getRoll() > 7) {
    } else {
      drivetrain.driveMecanum(speed, 0, 0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (drivetrain.getRoll() > 7) {
      return true;
    } else {
      return false;
    }
  }
}
