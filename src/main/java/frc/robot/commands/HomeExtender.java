package frc.robot.commands;

import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClawSystem;

public class HomeExtender extends CommandBase {
  private MedianFilter filter = new MedianFilter(3);
  private ClawSystem clawSystem;

  public HomeExtender(ClawSystem clawSystem) {
    addRequirements(clawSystem);
    this.clawSystem = clawSystem;
  }

  @Override
  public void initialize() {
    clawSystem.setExtenderOutputOverrideVolts(1.75);
  filter.reset();
  }

  @Override
  public void end(boolean interrupted) {
    clawSystem.setExtenderOutputOverrideVolts(null);
    clawSystem.zeroExtenderEncoder(.5);
    clawSystem.setExtendSetPoint(0);
  }

  @Override
  public boolean isFinished() {
    return filter.calculate(clawSystem.getExtenderCurrent()) > 20;
  
  }
}
