package frc.robot.commands;

import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.ClawSystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class DefaultAuto extends CommandBase{
    Drivetrain drivetrain;
    ClawSystem clawSystem;


    public DefaultAuto(Drivetrain drivetrain, ClawSystem clawSystem){

        addRequirements(drivetrain);
        this.drivetrain = drivetrain;
        this.clawSystem = clawSystem;

    }

        @Override
    public void initialize() { /* nothing */}

        @Override
    public void execute() { /* stuff */}

        @Override
    public void end(boolean interrupted){ /* nothing */}

        @Override
    public boolean isFinished() {
        return false;
    }
}