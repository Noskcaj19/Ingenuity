package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class Drive extends CommandBase {
  
  public Drive() {
    private Drivetrain drivetrain;
    public Command schedule;
    double distance = 0
  }

  @Override
  public void initialize() {
    drivetrain.zeroSensors();
  }

   public double setDistance(double distance){
    return distance;
   }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
