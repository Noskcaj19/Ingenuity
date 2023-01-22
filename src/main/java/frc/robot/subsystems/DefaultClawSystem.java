// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DefaultClawSystem extends SubsystemBase {
private CANSparkMax turnTable = new CANSparkMax(8, MotorType.kBrushed);
private CANSparkMax Arm = new CANSparkMax(9, MotorType.kBrushless);
private CANSparkMax Extender = new CANSparkMax(11, MotorType.kBrushless);


  /** Creates a new DefaultClawSystem. */
  public DefaultClawSystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void spinTable(double speed){
    turnTable.set(speed);
  }

  public void moveArm(double speed){
    Arm.set(speed);
  }

  public void extendArm(double speed){
    Extender.set(speed);
  }

}
