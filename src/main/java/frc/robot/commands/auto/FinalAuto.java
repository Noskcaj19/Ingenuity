// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Drive;
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

        addCommands(
                // new Drive(drivetrain, .5, .3),
                // new Turn(drivetrain, .3, 90, Direction.Clockwise),
                // new Turn(drivetrain, .3, 90, Direction.CounterClockwise),
                // new MoveArm(clawSystem, 140),
                // new MoveArm(clawSystem, -140)
                new ExtendArm(clawSystem, 20).withTimeout(.3) // This might not work because
        // idk how to get extend position
        // new MoveArm(clawSystem, .4).withTimeout(.3),
        // new ChangeClawStatus(clawSystem, ClawStatus.OPEN)
        // new TurnTable(clawSystem, 0, .4).withTimeout(.5),
        // new ChangeClawStatus(clawSystem, ClawStatus.CLOSE)
        );
        // SOME COMMANDS NEVER STOP WITHOUT TIMEOUT METHOD! DONT REMOVE IT!!!
        // WE NEED TO ADD SETPOINT TO TURNTABLE LATER

    }

}
