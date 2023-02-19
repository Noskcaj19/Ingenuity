// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClawSystem;

public class ChangeClawStatus extends CommandBase {
    enum ClawStatus {
        OPEN,
        CLOSE
    }

    private ClawSystem clawSystem;
    private ClawStatus status;
    private boolean isFinished;

    public ChangeClawStatus(ClawSystem clawSystem, ClawStatus status) {
        addRequirements(clawSystem);
        this.clawSystem = clawSystem;
        this.status = status;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        if (status == ClawStatus.OPEN)
            System.out.println("Claw started opening");
        else
            System.out.println("Claw started closing");
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (status == ClawStatus.OPEN)
            clawSystem.openClaw();
        else
            clawSystem.closeClaw();
        isFinished = true;
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        if (status == ClawStatus.OPEN)
            System.out.println("Claw stopped opening");
        else
            System.out.println("Claw stopped closing");
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return isFinished;
    }
}
