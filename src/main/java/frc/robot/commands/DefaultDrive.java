// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.auto.BalanceAuto;
import frc.robot.subsystems.Drivetrain;

public class DefaultDrive extends CommandBase {
    private XboxController primaryController;
    private Drivetrain drivetrain;
    private BalanceAuto balanceAuto;
    SlewRateLimiter filter = new SlewRateLimiter(0.5);
    SlewRateLimiter filterTurn = new SlewRateLimiter(1);

    // pretend that there is a second controller

    private final SlewRateLimiter speedLimiter = new SlewRateLimiter(5);
    private final SlewRateLimiter rotLimiter = new SlewRateLimiter(3);

    public DefaultDrive(Drivetrain drivetrain, XboxController primaryController) {
        addRequirements(drivetrain);
        this.primaryController = primaryController;
        this.drivetrain = drivetrain;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        final double xSpeed = speedLimiter.calculate(-primaryController.getLeftY());
        final double rot = rotLimiter.calculate(primaryController.getRightX());
        drivetrain.driveMecanum(
                xSpeed,
                primaryController.getLeftX() * 0,
                rot);
        if (primaryController.getStartButton()) {
            drivetrain.zero();
        }
        // drivetrain.driveMecanum(primaryController.getRightY() * .3,
        // primaryController.getLeftX() * .3,
        // primaryController.getRightX() * .3);
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
