// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Robot;
import frc.robot.subsystems.ClawSystem;
import frc.robot.commands.limelight.LimelightHorizAim;

public class DefaultTurret extends CommandBase {
    // Constant variables for speed
    private final double armExtendSpeed = .6;
    private final double TableSpinSpeed = .3;
    private final double armMoveSpeed = .2;

    private XboxController primaryController;
    private XboxController secondaryController;
    private ClawSystem clawSystem;
    private boolean claw = false;

    /** Creates a new DefaultTurret. */
    public DefaultTurret(ClawSystem clawSystem, XboxController primaryController, XboxController secondaryController) {
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(clawSystem);
        this.primaryController = primaryController;
        this.secondaryController = secondaryController;
        this.clawSystem = clawSystem;

    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        // code for extending arm
        if (primaryController.getAButton()) {
            clawSystem.setExtendSetPoint(armExtendSpeed);
        } else if (primaryController.getBButton()) {
            clawSystem.setExtendSetPoint(-armExtendSpeed);
        }

        // code for claw
        if (secondaryController.getAButtonPressed()) {

            if (claw == true) {
                clawSystem.openClaw();
                claw = !claw;
            } else if (claw == false) {
                clawSystem.closeClaw();
                claw = !claw;
            }
        }
        // old code for the turntable
        clawSystem.spinTablePID(
                clawSystem.getSpinTableSetpoint() + MathUtil.applyDeadband(secondaryController.getLeftX(), .1) * 8.5);

        if (secondaryController.getXButton()) {
            clawSystem.setGrabPoint();
        }

        double extendController = MathUtil.applyDeadband(secondaryController.getRightY() * 1.5, .03 * 2);
        var extendSet = -extendController + -clawSystem.getExtendSetPoint();
        clawSystem.setExtendSetPoint(extendSet);

        double moveController = MathUtil.applyDeadband(secondaryController.getLeftY(), .05) * 10;
        var scale = map(clawSystem.getExtendSetPoint(), -48, 0, .3, 1);
        var armSetpoint = (moveController * scale) + clawSystem.getArmSetPoint();
        clawSystem.setArmSetPoint(armSetpoint);

        if (secondaryController.getYButton()) {
            clawSystem.spinTablePID(0);
        }
    }

    static double map(double x, double in_min, double in_max, double out_min, double out_max) {
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
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
