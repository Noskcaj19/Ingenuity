// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.drive.MecanumDrive;

public class Drivetrain extends SubsystemBase {
  /** Creates a new Drivetrain. */
  private final WPI_TalonFX leftFront = new WPI_TalonFX(3);
  private final WPI_TalonFX rightRear = new WPI_TalonFX(2);
  private final WPI_TalonFX leftRear = new WPI_TalonFX(5);
  private final WPI_TalonFX rightFront = new WPI_TalonFX(4);

  private final MecanumDrive drivetrain = new MecanumDrive(leftFront, leftRear, rightFront, rightRear);
  private  AHRS navx;
  private Rotation2d initAngle;


public Drivetrain(){
  try {
    navx = new AHRS(Port.kMXP);
    // navx.setAngleAdjustment(navx.getRotation2d().getDegrees());
    initAngle = navx.getRotation2d();
  } catch (Exception ex) {}
  rightRear.setInverted(true);
  rightFront.setInverted(true);

  rightFront.configNominalOutputForward(0);
  rightRear.configNominalOutputForward(0);
  leftFront.configNominalOutputForward(0);
  leftRear.configNominalOutputForward(0);
  rightFront.configNominalOutputReverse(0);
  rightRear.configNominalOutputReverse(0);
  leftFront.configNominalOutputReverse(0);
  leftRear.configNominalOutputReverse(0);
}
  
  public void driveMecanum(double x, double y, double rotation) {
    System.out.println(navx.getRotation2d());
    drivetrain.driveCartesian(-x, y, rotation, navx.getRotation2d().minus(initAngle));
  }

  public void zero() {
    initAngle = navx.getRotation2d();
  }

  // public void drive(double x, double y, double z) {
  //   drivetrain.driveCartesian(-x, y, z);
  // }
}