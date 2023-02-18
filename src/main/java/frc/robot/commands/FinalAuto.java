// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Turn.Direction;
import frc.robot.subsystems.Drivetrain;

public class FinalAuto extends SequentialCommandGroup {
  /** Creates a new FinalAuto. */
  Drivetrain drivetrain;
  Drive drive;

  public FinalAuto(Drive drive, Drivetrain drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
    this.drivetrain = drivetrain;
    this.drive = drive;

    addCommands(
        new Drive(drivetrain, .5, .3),
        new Turn(drivetrain, .3, 90, Direction.CounterClockwise),
        new Turn(drivetrain, .3, 90, Direction.Clockwise));
    // new Drive(drivetrain, testDistance, -testAngle).withTimeout(.2),
    // new Drive(drivetrain, testDistance, .1).withTimeout(.1),
    // new Drive(drivetrain, testDistance, testAngle).withTimeout(.2),
    // new Drive(drivetrain, testDistance, .1).withTimeout(.1));

  }

}
