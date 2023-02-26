package frc.robot.subsystems;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.MathUtil;
import static frc.robot.Constants.LimelightConstants.*;

public class Limelight extends SubsystemBase {
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry ta = table.getEntry("ta");
    NetworkTableEntry tv = table.getEntry("tv");

    // Create variables for the different values given from the limelight
    private double xOffset; // Positive values mean that target is to the right of the camera; negative
    // values mean target is to the left. Measured in degrees

    private double yOffset; // Positive values mean that target is above the camera; negative values mean
    // target is below. Measured in degrees

    private double targetArea; // Returns a value of the percentage of the image the target takes

    private double targetValue; // Sends 1 if a target is detected, 0 if none are present
    // Create a network table for the limelight

    private MedianFilter distanceFilter = new MedianFilter(35);

    private NetworkTable limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
    private ShuffleboardTab limelightTab = Shuffleboard.getTab("Limelight data");
    //

    public Limelight() {

        limelightTable.getEntry("pipeline").setNumber(0);

    }

    @Override
    public void periodic() {

    }

    public double getXOffset() {
        return xOffset;
    }

    public void limelightLightOff() {
        table.getEntry("ledMode").setNumber(1);
    }

    public void limelightLightOn() {
        table.getEntry("ledMode").setNumber(0);
    }

    public double getX() {
        double x = tx.getDouble(0.0);
        return x;
    }

    public double getY() {
        double y = ty.getDouble(0.0);

        return y;
    }

    public double getArea() { // gets area of the april tags
        double area = ta.getDouble(0.0);
        return area;
    }

    public boolean getTV() {
        int target = tv.getNumber(0).intValue();
        return target == 1;
    }

    public boolean isTargetDetected() { // keeps track of whether target is detected
        return (targetValue > 0.0);
    }

    public boolean isTargetCentered() { // Keeps track of whether target is centered
        return ((xOffset > -1.5) && (xOffset < 1.5) && (xOffset != 0.0));
    }

    public double limelightDistance() {
        return (kPortHeight - kLimelightHeight)
                / Math.tan(Math.toRadians(kLimelightAngle + yOffset) + kLimelightOffset);
    }

    public double filteredLimelightDistance() {
        return distanceFilter.calculate(limelightDistance());
    }

    public double getPipeline() {
        NetworkTableEntry pipeline = limelightTable.getEntry("Pipeline");
        return pipeline.getDouble(0.0);
    }

    public void setPipeline(int pipeline) {
        int clampedPipeline = MathUtil.clamp(pipeline, 0, 9);
        limelightTable.getEntry("Pipeline").setValue(clampedPipeline);
    }
    // public void enableLeds() {
    // table.
    // }

    // public void disableLeds() {
    // limelightLedEntry.setNumber(LimelightConstants.kLedDisabled);
    // }

    // public void blinkLeds() {
    // limelightLedEntry.setNumber(LimelightConstants.kLedBlink);
    // }

    public void updateLimelight() {
        // Updates the values of the limelight on the network table
        xOffset = limelightTable.getEntry("tx").getDouble(0.0);
        yOffset = limelightTable.getEntry("ty").getDouble(0.0);
        targetArea = limelightTable.getEntry("ta").getDouble(0.0);
        targetValue = limelightTable.getEntry("tv").getDouble(0.0);
    }

    public void log() {
        // Updates the SmartDashboard with limelight values
        // limelightXOffsetEntry.setNumber(xOffset);
        // limelightYOffsetEntry.setNumber(yOffset);

        // limelightAreaPercentageEntry.setNumber(targetArea);
        // limelightTargetCenteredEntry.setBoolean(isTargetCentered());
        // limelightTargetDetectedEntry.setBoolean(isTargetDetected());
        // limelightDistanceEntry.setNumber(limelightDistance());
    }

    public static final class LimelightConstants {
        public static final int kLedDisabled = 1;
        public static final int kLedBlink = 2;
        public static final int kLedEnabled = 3;
    }
}