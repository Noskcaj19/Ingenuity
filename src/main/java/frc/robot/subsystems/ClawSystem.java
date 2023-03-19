// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PneumaticsControlModule;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

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
	private final PIDController armPID = new PIDController(0.05, 0, 0.001);
	private final PIDController extendPID = new PIDController(0.2, 0, 0.001);
	private final PIDController turnPID = new PIDController(0.010, 0, 0.012 * 0.2);

	private final SlewRateLimiter turnLimiter = new SlewRateLimiter(2);

	private TurretMode turretMode = TurretMode.Power;
	private double turretSpeed = 0;

	/** Creates a new ClawSystemp. */
	public ClawSystem(PneumaticsControlModule pCM) {
		this.pCM = pCM;
		armSolenoid = pCM.makeDoubleSolenoid(4, 5);
		// armSolenoid.set(Value.kForward);
		Shuffleboard.getTab("Debug").add(armPID);
		Shuffleboard.getTab("Debug").add(extendPID);

		armPID.setTolerance(5);
		extendPID.setTolerance(2);
	}

	@Override
	public void periodic() {
		// System.out.println("Arm Target " + armPID.getSetpoint() + " actual " +
		// arm2.getDistance());
		// This method will be called once per scheduler run

		// System.out.println("TURRET VAL" + turnCoder.getDistance());

		arm.set(-MathUtil.clamp(armPID.calculate(/* arm */arm2.getDistance()), -1, 1));
		extender.set(
				MathUtil.clamp(extendPID.calculate(extender.getEncoder().getPosition()), -0.7, 0.7));
		var pidOut = turnPID.calculate(-turnCoder.getDistance());
		var slewOut = turnLimiter.calculate(pidOut);
		if (turretMode == TurretMode.Pid) {
			turnTable.set(slewOut);
		} else {
			turnTable.set(turretSpeed);
		}
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
		turnPID.setSetpoint(MathUtil.clamp(setpoint, -250, 250));
		System.out.println("Turn setpoint: " + setpoint + " encoder: " +
				turnCoder.getDistance());
	}

	public double getSpinTableSetpoint() {
		return turnPID.getSetpoint();
	}

	public double getExtendSetPoint() {
		return extendPID.getSetpoint();
	}

	public double getArmSetPoint() {
		return armPID.getSetpoint();
	}

	// public void setTurnSetPoint(double setpoint) {
	// System.out.println("Setting turn setpoint to: " + setpoint);
	// turnPID.setSetpoint(setpoint); // add clamps later maybe
	// }

	public void setGrabPoint() {
		armPID.setSetpoint(-1014.45); // align to y of h player station
	}

	public void setExtendSetPoint(double setpoint) {
		extendPID.setSetpoint(-MathUtil.clamp(setpoint, 0, 48));
	}

	public void setArmSetPoint(double setpoint) {
		armPID.setSetpoint(MathUtil.clamp(setpoint, -1480, 10));
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

}
