// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClawSystem;

public class ExtendArm extends CommandBase {

    ClawSystem clawSystem;
    private double setpoint;

    public ExtendArm(ClawSystem clawSystem, double setpoint) {
        addRequirements(clawSystem);
        this.setpoint = MathUtil.clamp(setpoint, 0, 40);

    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        System.out.println("Arm started extending");
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        clawSystem.extendArm(setpoint);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        System.out.println("Arm stopped extending");
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        if (clawSystem.getArmExtendPosition() == setpoint)
            return true;
        return false;
    }
}
