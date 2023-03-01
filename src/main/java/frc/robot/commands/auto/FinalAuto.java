// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.util.sendable.*;
import frc.robot.commands.auto.Drive;
import frc.robot.commands.auto.ChangeClawStatus.ClawStatus;
import frc.robot.commands.auto.Turn.Direction;
import frc.robot.subsystems.ClawSystem;
import frc.robot.subsystems.Drivetrain;

public class FinalAuto extends SequentialCommandGroup {
    /** Creates a new FinalAuto. */
    Drivetrain drivetrain;
    ClawSystem clawSystem;
    Drive drive;
  

    public FinalAuto(Drive drive, Drivetrain drivetrain, ClawSystem clawSystem) {
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(drivetrain);
        addRequirements(clawSystem);
        this.drivetrain = drivetrain;
        this.clawSystem = clawSystem;
        this.drive = drive;
        //int mode = 1;
    }

        public void mode1() {
        //if (mode == 1)

        //{
            // Mode 1 for going to another cone after
            System.out.println("Mode 1 starts");
            addCommands(
                    // comment for other people
                    new ChangeClawStatus(clawSystem, ClawStatus.CLOSE),
                    new MoveArm(clawSystem, 440),
                    new Drive(drivetrain, 0.5969, .2),
                    new ChangeClawStatus(clawSystem, ClawStatus.OPEN),
                    new Drive(drivetrain, 0.5969, .2),
                    new Turn(drivetrain, .5, 180, Direction.CounterClockwise),
                    new BalanceAuto(drivetrain, -0.3),
                    new BalanceAutoPartTwo(drivetrain, -0.2).withTimeout(2));
        }
        //} else if (mode == 2) {

        public void mode2() {

            // Mode 2 for going to charging station after
            addCommands(
                    new ChangeClawStatus(clawSystem, ClawStatus.CLOSE),
                    new MoveArm(clawSystem, 440),
                    new Drive(drivetrain, 0.5969, .2),
                    new ChangeClawStatus(clawSystem, ClawStatus.OPEN),
                    new Drive(drivetrain, -0.5, 1),
                    new Turn(drivetrain, .5, 90, Direction.CounterClockwise),
                    new Drive(drivetrain, 0.5, 1),
                    new Turn(drivetrain, .5, 90, Direction.CounterClockwise),
                    new BalanceAuto(drivetrain, -0.2),
                    new BalanceAutoPartTwo(drivetrain, -0.2).withTimeout(0.4));

        }

    }
