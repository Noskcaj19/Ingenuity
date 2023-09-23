// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.EnumSet;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PneumaticsControlModule;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.InterpLUT;

public class ClawSystem extends SubsystemBase {
	public enum TurretMode {
		Pid,
		Power
	}

	private CANSparkMax turnTable = new CANSparkMax(8, MotorType.kBrushed);
	private CANSparkMax arm = new CANSparkMax(9, MotorType.kBrushed);
	private CANSparkMax extender = new CANSparkMax(11, MotorType.kBrushless);
	private CANSparkMax roller = new CANSparkMax(30, MotorType.kBrushed);
	private PneumaticsControlModule pCM;
	private final DoubleSolenoid armSolenoid;
	private final Encoder arm2 = new Encoder(0, 1);
	private final Encoder turnCoder = new Encoder(2, 3);

	// p 0.5
	// good values for position controll p 0.4 i 0 d 0.002
	GenericEntry armEleMaxVel = Shuffleboard.getTab("Debug").getLayout("Arm Elevation Control", BuiltInLayouts.kList)
			.addPersistent("Max Vel", 1100.0).getEntry();
	GenericEntry armEleMaxAccel = Shuffleboard.getTab("Debug").getLayout("Arm Elevation Control", BuiltInLayouts.kList)
			.addPersistent("Max Accel", 1600.0).getEntry();
	private final ProfiledPIDController armPID = new ProfiledPIDController(
			0.05,
			0,
			0.001,
			new TrapezoidProfile.Constraints(armEleMaxVel.getDouble(1100), armEleMaxAccel.getDouble(1500)));
	private final PIDController extendPID = new PIDController(0.2, 0, 0.001);
	// GenericEntry turnMaxVel = Shuffleboard.getTab("Debug").getLayout("Turn Control", BuiltInLayouts.kList)
	// 		.addPersistent("Max Vel", 100.0).getEntry();
	// GenericEntry turnMaxAccel = Shuffleboard.getTab("Debug").getLayout("Turn Control", BuiltInLayouts.kList)
	// 		.addPersistent("Max Accel", 100.0).getEntry();
	private final double turnPIDMaxVel = 1100;
	// private final double turnPIDMaxAccel = 1100;
	private final ProfiledPIDController turnPID = new ProfiledPIDController(
			0.20448, 0.0, 0.098755,
			// 500, 1100
			new TrapezoidProfile.Constraints(turnPIDMaxVel, 750));

	LinearFilter turnEncoderFilter = LinearFilter.singlePoleIIR(0.13, 0.02);

	private final SimpleMotorFeedforward turnFF = new SimpleMotorFeedforward(1.0337, 0.019395, 0.01235);
	private final ArmFeedforward armFF = new ArmFeedforward(-0.061147, 0.1, 0.13446, 0.019562);

	private final SlewRateLimiter turnLimiter = new SlewRateLimiter(2);

	private TurretMode turretMode = TurretMode.Power;
	private double turretSpeed = 0;

	private double lastArmSpeed = 0;
	private double lastArmTime = Timer.getFPGATimestamp();
	private double lastTurretSpeed = 0;
	private double lastTurretTime = Timer.getFPGATimestamp();

	private final InterpLUT armTurretMaxAccelLUT;
	private final ControlTelemetry turnTM;
	private final ControlTelemetry armTM;

	private Double extenderOutputOverride = null;
	private Double wristOutputOverride = null;

	public void setExtenderOutputOverrideVolts(Double extenderOutputOverride) {
		this.extenderOutputOverride = extenderOutputOverride;
	}

	public void setWristOutputOverrideVolts(Double wristOutputOverride) {
		this.wristOutputOverride = wristOutputOverride;
	}

	private final class ControlTelemetry {
		private final GenericEntry in;
		private final GenericEntry inRaw;
		private final GenericEntry effect;
		private final GenericEntry setpoint;
		private final GenericEntry goal;
		private final GenericEntry ff;
		private final GenericEntry accel;

		ControlTelemetry(ShuffleboardLayout layout, String prefix) {
			inRaw = layout.add(prefix + " In Raw", 0).getEntry();
			in = layout.add(prefix + " In", 0).getEntry();
			effect = layout.add(prefix + " Effect", 0).getEntry();
			setpoint = layout.add(prefix + " Setpoint", 0).getEntry();
			goal = layout.add(prefix + " Goal", 0).getEntry();
			ff = layout.add(prefix + " FF", 0).getEntry();
			accel = layout.add(prefix + " Accel", 0).getEntry();
		}
	}

	/** Creates a new ClawSystemp. */
	public ClawSystem(PneumaticsControlModule pCM) {
		// NetworkTableInstance.getDefault().addListener(
		// armEleMaxAccel.getTopic(), EnumSet.of(NetworkTableEvent.Kind.kValueAll),
		// event -> {
		// armPID.setConstraints(new
		// TrapezoidProfile.Constraints(armEleMaxVel.getDouble(0),
		// event.valueData.value.getDouble()));
		// });
		// NetworkTableInstance.getDefault().addListener(
		// armEleMaxVel.getTopic(), EnumSet.of(NetworkTableEvent.Kind.kValueAll), event
		// -> {
		// armPID.setConstraints(new
		// TrapezoidProfile.Constraints(event.valueData.value.getDouble(),
		// armEleMaxAccel.getDouble(0)));
		// });

		// NetworkTableInstance.getDefault().addListener(
		// turnMaxAccel.getTopic(), EnumSet.of(NetworkTableEvent.Kind.kValueAll), event
		// -> {
		// turnPID.setConstraints(new
		// TrapezoidProfile.Constraints(turnMaxVel.getDouble(0),
		// event.valueData.value.getDouble()));
		// });
		// NetworkTableInstance.getDefault().addListener(
		// turnMaxVel.getTopic(), EnumSet.of(NetworkTableEvent.Kind.kValueAll), event ->
		// {
		// turnPID.setConstraints(new
		// TrapezoidProfile.Constraints(event.valueData.value.getDouble(),
		// turnMaxAccel.getDouble(0)));
		// });

		this.pCM = pCM;
		armSolenoid = pCM.makeDoubleSolenoid(4, 5);
		// armSolenoid.set(Value.kForward);
		ShuffleboardTab debugTab = Shuffleboard.getTab("Debug");
		debugTab.add(armPID);
		debugTab.add(extendPID);

		var turnControlLayout = debugTab.getLayout("Turn Control", BuiltInLayouts.kList);
		turnControlLayout.add(turnPID);
		turnTM = new ControlTelemetry(turnControlLayout, "Turn");

		var armPitchControlLayout = debugTab.getLayout("Arm Elevation Control", BuiltInLayouts.kList);
		armPitchControlLayout.add(armPID);
		armTM = new ControlTelemetry(armPitchControlLayout, "Arm Elevation");

		armPID.setTolerance(5);
		extendPID.setTolerance(2);

		// turnCoder.setDistancePerPulse(1/1166.666666);

		armTurretMaxAccelLUT = new InterpLUT();
		// extended
		armTurretMaxAccelLUT.add(-48, 450);
		// retracted
		armTurretMaxAccelLUT.add(0, 750);
		armTurretMaxAccelLUT.createLUT();
	}

	@Override
	public void periodic() {
		// System.out.println("Arm Target " + armPID.getSetpoint() + " actual " +
		// arm2.getDistance());
		// This method will be called once per scheduler run

		// System.out.println("TURRET VAL" + turnCoder.getDistance());

		// - Gain scheduling -
		// double armPLUTVal = armPLUT.get(arm2.getDistance());
		// double armDLUTVal = armDLUT.get(arm2.getDistance());
		// armPLUTEntry.setDouble(armPLUTVal);
		// armDLUTEntry.setDouble(armDLUTVal);

		// armPID.setP(armPLUTVal);
		// armPID.setD(armDLUTVal);

		var clampedArmEffect = MathUtil.clamp(-armPID.calculate(/* arm */arm2.getDistance()), -1, 1);
		armTM.inRaw.setDouble(arm2.getDistance());

		armTM.effect.setDouble(clampedArmEffect);
		armTM.goal.setDouble(armPID.getGoal().position);
		armTM.setpoint.setDouble(armPID.getSetpoint().position);

		if (wristOutputOverride == null) {
			arm.set(clampedArmEffect);
		} else {
			arm.set(wristOutputOverride);
		}

		// // FF
		// double armAcceleration = (armPID.getSetpoint().velocity - lastArmSpeed)
		// / (Timer.getFPGATimestamp() - lastArmTime);
		// double armFFVal = armFF.calculate(armPID.getSetpoint().velocity,
		// armAcceleration);
		// armTM.ff.setDouble(armFFVal);
		// armTM.accel.setDouble(armAcceleration);
		// arm.setVoltage(clampedArmEffect + armFFVal);
		// lastArmSpeed = armPID.getSetpoint().velocity;
		// lastArmTime = Timer.getFPGATimestamp();

		// --- Extender ---
		if (extenderOutputOverride == null) {
			extender.set(
					MathUtil.clamp(extendPID.calculate(extender.getEncoder().getPosition()), -0.7, 0.7));
		} else {
			extender.setVoltage(extenderOutputOverride);
		}

		// --- Turn table ---
		// - Gain scheduling -
		// double turnPLUTVal = turnPLUT.get(-turnCoder.getDistance());
		// double turnDLUTVal = turnDLUT.get(-turnCoder.getDistance());
		// turretPLUTEntry.setDouble(turnPLUTVal);
		// turretDLUTEntry.setDouble(turnDLUTVal);

		// turnPID.setP(turnPLUTVal);
		// turnPID.setD(turnDLUTVal);

		turnPID.setConstraints(new TrapezoidProfile.Constraints(turnPIDMaxVel,
				armTurretMaxAccelLUT.get(extender.getEncoder().getPosition())));

		turnTM.inRaw.setDouble(-turnCoder.getDistance());
		var filteredTurnEncoder = turnEncoderFilter.calculate(-turnCoder.getDistance());
		turnTM.in.setDouble(filteredTurnEncoder);
		var turnPidOut = turnPID.calculate(filteredTurnEncoder);
		turnTM.effect.setDouble(turnPidOut);
		// var slewOut = turnLimiter.calculate(pidOut);
		turnTM.setpoint.setDouble(turnPID.getSetpoint().position);
		turnTM.goal.setDouble(turnPID.getGoal().position);

		// turnTable.set(turnPidOut + (0.05* Math.signum(turnPidOut) + (0.0 *
		// turnPID.getSetpoint().velocity)));

		// FF
		double turnAcceleration = (turnPID.getSetpoint().velocity - lastTurretSpeed)
				/ (Timer.getFPGATimestamp() - lastTurretTime);
		double turnFFVal = turnFF.calculate(turnPID.getSetpoint().velocity,
				turnAcceleration);
		turnTM.ff.setDouble(turnFFVal);
		turnTM.accel.setDouble(turnAcceleration);
		turnTable.setVoltage(turnPidOut + turnFFVal);
		lastTurretSpeed = turnPID.getSetpoint().velocity;
		lastTurretTime = Timer.getFPGATimestamp();
	}

	public void setTurretMode(TurretMode mode) {
		turretMode = mode;
	}

	public boolean extendAtSetpoint() {
		return extendPID.atSetpoint();
	}

	public boolean armAtSetpoint() {
		return armPID.atSetpoint();
	}

	public void spinTable(double speed) {
		turretSpeed = speed;
	}

	public void spinTablePID(double setpoint) {
		// turnPID.setGoal(MathUtil.clamp(setpoint, -250, 250));
		turnPID.setGoal((setpoint));
	}

	public double getSpinTableSetpoint() {
		return turnPID.getGoal().position;
	}

	public double getExtendSetPoint() {
		return extendPID.getSetpoint();
	}

	public double getArmSetPoint() {
		return armPID.getGoal().position;
	}

	// public void setTurnSetPoint(double setpoint) {
	// System.out.println("Setting turn setpoint to: " + setpoint);
	// turnPID.setSetpoint(setpoint); // add clamps later maybe
	// }

	public void setGrabPoint() {
		armPID.setGoal(-1014.45); // align to y of h player station
	}

	public void setExtendSetPoint(double setpoint) {
		extendPID.setSetpoint(-MathUtil.clamp(setpoint, 0, 48));
	}

	public void setArmSetPoint(double setpoint) {
		armPID.setGoal(MathUtil.clamp(setpoint, -1480, 10));
		// armPID.setSetpoint(setpoint);
	}

	public void openClaw() {
		armSolenoid.set(Value.kForward);
	}

	public void closeClaw() {
		armSolenoid.set(Value.kReverse);
	}

	public void rollerIn() {
		roller.set(0.5);
	}

	public void rollerOut() {
		roller.set(-0.5);
	}

	public void rollerStop() {
		roller.set(0);
	}

	public double getExtenderCurrent() {
		return extender.getOutputCurrent();
	}

	public void zeroExtenderEncoder(double offset) {
		extender.getEncoder().setPosition(offset);
	}

	public double getWristCurrent() {
		return arm.getOutputCurrent();
	}

	public void zeroWristEncoder() {
		arm2.reset();
	}

}
