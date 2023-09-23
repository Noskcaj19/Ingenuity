package frc.robot.commands;

import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClawSystem;

public class HomeElevator extends CommandBase {
  private MedianFilter filter = new MedianFilter(3);
  private ClawSystem clawSystem;

  public HomeElevator(ClawSystem clawSystem) {
    addRequirements(clawSystem);
    this.clawSystem = clawSystem;
  }

  @Override
  public void initialize() {
    clawSystem.setWristOutputOverrideVolts(-.4);
    filter.reset();
  }

  @Override
  public void end(boolean interrupted) {
    clawSystem.setWristOutputOverrideVolts(null);
    clawSystem.zeroWristEncoder();
    clawSystem.setArmSetPoint( 10);
  }

  @Override
  public boolean isFinished() {
    System.out.println(clawSystem.getWristCurrent());
    return filter.calculate(clawSystem.getWristCurrent()) > 3.5;

  }
}
