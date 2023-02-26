package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Limelight;
import frc.robot.Constants;
import frc.robot.subsystems.ClawSystem;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.Constants;

 
public class AutoLimelight extends CommandBase{
    private ClawSystem clawSystem;
    private final PIDController turnPid;
    private final PIDController distancePid;
    boolean turnOff = false;
    private Limelight limelight;
    private XboxController primaryController;
    private Drivetrain drivetrain;


    public AutoLimelight(ClawSystem clawSystem, XboxController primaryController, Limelight limelight, boolean turnOff, Drivetrain drivetrain){
        addRequirements(clawSystem);
        this.clawSystem = clawSystem;
        this.limelight = limelight;
        this.turnOff = turnOff;
        this.primaryController = primaryController;
        turnPid = new PIDController(0.0525, 0.009, 0.0001);
        turnPid.setSetpoint(0);
        distancePid = new PIDController(0.0225, 0.0, 0.0001);
        distancePid.setSetpoint(-70);

        Shuffleboard.getTab("Debug").add("Limelight turn", turnPid);
        this.drivetrain = drivetrain;
    }


    @Override
    public void initialize(){
    }

    @Override
    public void execute(){

        if (limelight.isTargetDetected()) {
            double rotOut = turnPid.calculate(limelight.getXOffset());

            double distance = limelight.filteredLimelightDistance();
            double distancePidOut = MathUtil.clamp(distancePid.calculate(distance),
                    -.4 * Constants.kMaxSpeed,
                    .4 * Constants.kMaxSpeed);
            drivetrain.driveMecanum(-(distancePidOut), -rotOut, 0);


        } else {
            drivetrain.driveMecanum(0, 0, 0);
        }

    }
    

}

