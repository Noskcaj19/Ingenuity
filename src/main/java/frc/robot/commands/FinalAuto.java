// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drivetrain;

public class FinalAuto extends CommandBase {
    /** Creates a new FinalAuto. */
    Drivetrain drivetrain;
    Drive drive;

    public FinalAuto(Drive drive, Drivetrain drivetrain) {
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(drivetrain);
        this.drivetrain = drivetrain;
        this.drive = drive;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // new Drive(drivetrain, .8).schedule();
        double testDistance = .8;
        double testAngle = 90;
        new SequentialCommandGroup(
                new Drive(drivetrain, testDistance, .1).withTimeout(.1),
                new Drive(drivetrain, testDistance, -testAngle).withTimeout(.2),
                new Drive(drivetrain, testDistance, .1).withTimeout(.1),
                new Drive(drivetrain, testDistance, testAngle).withTimeout(.2),
                new Drive(drivetrain, testDistance, .1).withTimeout(.1));

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
