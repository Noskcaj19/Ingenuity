// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.MecanumDrive;

public class Drivetrain extends SubsystemBase {
  /** Creates a new Drivetrain. */
  private final WPI_TalonFX leftFront = new WPI_TalonFX(2);
  private final WPI_TalonFX rightRear = new WPI_TalonFX(3);
  private final WPI_TalonFX leftRear = new WPI_TalonFX(4);
  private final WPI_TalonFX rightFront = new WPI_TalonFX(5);

  private final MecanumDrive drivetrain = new MecanumDrive(leftFront, leftRear, rightFront, rightRear);

  public void driveMecanum(double x, double y, double rotation) {
    drivetrain.driveCartesian(y, -x, -rotation);
  }

  public void drive(double x, double y, double z) {
    drivetrain.driveCartesian(-x, y, z);
  }
}