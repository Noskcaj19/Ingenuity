package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class Drive extends CommandBase {

private Drivetrain drivetrain;
private double distance;

    public Drive(Drivetrain drivetrain, double distance) {
        addRequirements(drivetrain);
        
        this.drivetrain = drivetrain;
        this.distance = distance;
    }

    @Override
    public void initialize() {
        drivetrain.zeroSensors();
    }

    public double setDistance(double distance){
        return distance;
    }

    @Override
    public void execute() {
    // if the robot has not reached the set distance, keep driving
    // otherwise, (if the distance has been reached), stop
        if (drivetrain.getDistance() < distance){
            drivetrain.driveMecanum(0, 0.1, 0);
        } else {
            drivetrain.driveMecanum(0, 0, 0);
        }
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
    // if the distance has been reached, the command is finished
    // otherwise, the command is not finished
        if (drivetrain.getDistance() > distance){
            return true;
        } else {
            return false;
        }
    }
}