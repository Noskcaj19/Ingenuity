// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PneumaticsControlModule;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClawSystem extends SubsystemBase {
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
	private final PIDController turnPID = new PIDController(0, 0, 0);

	/** Creates a new ClawSystem. */
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
		System.out.println("Arm Target " + armPID.getSetpoint() + " actual " +
				arm2.getDistance());
		// This method will be called once per scheduler run

		arm.set(-MathUtil.clamp(armPID.calculate(/* arm */arm2.getDistance()), -1, 1));
		extender.set(
				MathUtil.clamp(extendPID.calculate(extender.getEncoder().getPosition()), -0.7, 0.7));
	}

	public boolean extendAtSetpoint() {
		return extendPID.atSetpoint();
	}

	public boolean armAtSetpoint() {
		return armPID.atSetpoint();
	}

	public void spinTable(double speed) {
		turnTable.set(speed);
	}

	public void spinTablePID(double setpoint) {
		turnTable.set(turnPID.calculate(turnCoder.getDistance(), setpoint));
	}

	public double getExtendSetPoint() {

		return extendPID.getSetpoint();
	}

	public double getArmSetPoint() {

		return armPID.getSetpoint();
	}

	public void setGrabPoint() {
		armPID.setSetpoint(-1014.45);
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
