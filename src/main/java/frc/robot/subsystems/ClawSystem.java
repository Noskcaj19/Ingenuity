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
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClawSystem extends SubsystemBase {
	private CANSparkMax turnTable = new CANSparkMax(8, MotorType.kBrushed);
	private CANSparkMax arm = new CANSparkMax(9, MotorType.kBrushed);
	private CANSparkMax extender = new CANSparkMax(11, MotorType.kBrushless);
	private CANSparkMax roller = new CANSparkMax(30, MotorType.kBrushed);
	private PneumaticsControlModule pCM;
	private final DoubleSolenoid armSolenoid;
	private final Encoder arm2 = new new Encoder(0, 1);

	//p 0.5
	//good values for position controll p 0.4 i 0 d 0.002
	private final PIDController armPID = new PIDController(0.4, 0, 0.002);
	private final PIDController extendPID = new PIDController(0.3, 0, 0);

	/** Creates a new ClawSystem. */
	public ClawSystem(PneumaticsControlModule pCM) {
		this.pCM = pCM;
		armSolenoid = pCM.makeDoubleSolenoid(6, 7);
		//armSolenoid.set(Value.kForward);
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
	}

	public void spinTable(double speed){
		turnTable.set(speed);
	}

	public void moveArm(double setpoint){
		arm.set(MathUtil.clamp(armPID.calculate(/*arm*/arm2.getEncoder().getPosition(), setpoint), -0.5, 0.5));
		System.out.println(MathUtil.clamp(armPID.calculate(arm2.getEncoder().getPosition(), setpoint), -0.5, 0.5));
	}

	public void extendArm(double extendSetpoint){
		extender.set(MathUtil.clamp(extendPID.calculate(extender.getEncoder().getPosition(), extendSetpoint), -0.7, 0.7));
	}

	public void openClaw(){
		armSolenoid.set(Value.kForward);
	}

	public void closeClaw(){
		armSolenoid.set(Value.kReverse);
	}

	public void rollerIn(){
		roller.set(0.5);
	}

	public void rollerOut(){
		roller.set(-0.5);
	}

	public void rollerStop(){
		roller.set(0);
	}

}
