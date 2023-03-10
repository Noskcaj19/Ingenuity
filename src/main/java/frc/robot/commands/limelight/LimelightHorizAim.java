package frc.robot.commands.limelight;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Limelight;
import frc.robot.Constants;
import frc.robot.subsystems.ClawSystem;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.XboxController;

public class LimelightHorizAim extends CommandBase {
    private ClawSystem clawSystem;
    private Limelight limelight;
    private XboxController secondaryController;
    private Drivetrain drivetrain;
    private final PIDController turnPid;

    public LimelightHorizAim(ClawSystem clawSystem, Limelight limelight,
            XboxController secondaryController, Drivetrain drivetrain) {

        this.clawSystem = clawSystem;
        this.limelight = limelight;
        this.secondaryController = secondaryController;
        this.drivetrain = drivetrain;
        turnPid = new PIDController(0.01, 0, 0);

    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        if (limelight.isTargetDetected()) {
            double rotOut = turnPid.calculate(limelight.getXOffset());

            clawSystem.spinTablePID(rotOut);

            System.out.println(rotOut);
            // alt idea:
            // sp = clawSystem.getTurnSetPoint();
            // clawSystem.setTurnSetPoint(ap+rotOut);
            // should set the turntable setpoint to the displacement of the target
        } else {
            System.out.println("No target");
        }
    }

}