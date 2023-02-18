// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClawSystem;

public class DefaultTurret extends CommandBase {
    // Constant variables for speed
    private final double armExtendSpeed = .6;
    private final double TableSpinSpeed = .3;
    private final double armMoveSpeed = .2;

    private double extendController = 0;
    private double moveController = 0;
    private double extendSet = 0;
    private double set = 0;

    private XboxController primaryController;
    private XboxController secondaryController;
    private ClawSystem clawSystem;

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
        System.out.println(set);
        // code for turntable
        if (primaryController.getRightBumper()) {
            clawSystem.spinTable(TableSpinSpeed);
        } else if (primaryController.getLeftBumper()) {
            clawSystem.spinTable(-TableSpinSpeed);
        } else {
            clawSystem.spinTable(0);
        }

        // code for extending arm
        if (primaryController.getAButton()) {
            clawSystem.extendArm(armExtendSpeed);
        } else if (primaryController.getBButton()) {
            clawSystem.extendArm(-armExtendSpeed);
        } else {
            clawSystem.extendArm(0);
        }

        // code for claw
        if (secondaryController.getAButtonPressed()) {
            clawSystem.openClaw();
        }
        if (secondaryController.getBButtonPressed()) {
            clawSystem.closeClaw();
        }

        // code for turning the roller on and off :3
        if (secondaryController.getXButtonPressed()) {
            clawSystem.rollerIn();
        }
        if (secondaryController.getXButtonReleased()) {
            clawSystem.rollerStop();
        }
        if (secondaryController.getYButtonPressed()) {
            clawSystem.rollerOut();
        }
        if (secondaryController.getYButtonReleased()) {
            clawSystem.rollerStop();
        }

        // extending arm on second controller
        // if (secondaryController.getRightBumper()) {
        // extendSet = MathUtil.clamp(extendSet + 0.5, 0, 45);
        // clawSystem.extendArm(extendSet);
        // } else if (secondaryController.getLeftBumper()){
        // extendSet = MathUtil.clamp(extendSet - 0.5, 0, 45);
        // clawSystem.extendArm(extendSet);
        // }
        // System.out.println(extendSet);
        // clawSystem.extendArm(extendSet);

        // our own very special deadband method!!!
        if (secondaryController.getRightY() < 0.03 && secondaryController.getRightY() > -0.03) {
            extendController = 0;
        } else {
            extendController = secondaryController.getRightY();
        }

        if (secondaryController.getLeftY() < 0.03 && secondaryController.getLeftY() > -0.03) {
            moveController = 0;
        } else {
            moveController = secondaryController.getLeftY();
        }

        extendSet = MathUtil.clamp(-extendController + extendSet, 0, 40);
        clawSystem.extendArm(extendSet);
        // else {
        // clawSystem.extendArm(extendSet);
        // }

        // //sets setpoint for PID
        // if(secondaryController.getXButton()){
        // set = set + 0.1;
        // }
        // if(secondaryController.getYButton()){
        // set = set - 0.1;
        // }
        // secondarycontroller > 0.1 thing

        clawSystem.spinTable(secondaryController.getLeftX() / 2);
        // clawSystem.moveArm(secondaryController.getLeftX()*1/50);
        // clawSystem.moveArm(set);

        set -= moveController * 10; // Changed from: set = (-moveController / 4) + set; I hope it works
        set = MathUtil.clamp(set, -580.0, 450.0);
        clawSystem.moveArm(set);

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
