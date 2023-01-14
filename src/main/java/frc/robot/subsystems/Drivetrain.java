// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveOdometry;

public class Drivetrain extends SubsystemBase {
  /** Creates a new Drivetrain. */
  private final WPI_TalonFX leftFront = new WPI_TalonFX(3);
  private final WPI_TalonFX rightRear = new WPI_TalonFX(2);
  private final WPI_TalonFX leftRear = new WPI_TalonFX(5);
  private final WPI_TalonFX rightFront = new WPI_TalonFX(4);
  
  Encoder encoder = new Encoder(0,1);

  Translation2d m_frontLeftLocation = new Translation2d(0.305, 0.257175);
  Translation2d m_frontRightLocation = new Translation2d(-0.305, 0.257175);
  Translation2d m_backLeftLocation = new Translation2d(0.305, -0.257175);
  Translation2d m_backRightLocation = new Translation2d(-0.305, -0.257175);  

  public static final double kGearRatio = 10.71;
  public static final double kWheelRadius = 0.076;
  public static final double kEncoderResolution = 2048;
  public static final double kDistancePerTick = (2 * Math.PI * kWheelRadius / kEncoderResolution) / kGearRatio;



  private final MecanumDriveKinematics kinematics = new MecanumDriveKinematics(m_backLeftLocation, m_backRightLocation, m_frontLeftLocation, m_frontRightLocation);

  private final MecanumDrive drivetrain = new MecanumDrive(leftFront, leftRear, rightFront, rightRear);
  private  AHRS navx;
  private Rotation2d initAngle;

  //private final MecanumDriveOdometry odometry = new MecanumDriveOdometry(kinematics, navx.getRotation2d(), new Mechanism2d(rightFront.getse)



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
  // odometry = new MechanumDriveOdomotry
}
  
  public void driveMecanum(double x, double y, double rotation) {
    System.out.println(navx.getRotation2d());
    drivetrain.driveCartesian(-x, y, rotation, navx.getRotation2d().minus(initAngle));
  }

  public void zero() {
    initAngle = navx.getRotation2d();
  }

  public double getDistance() {
    double leftMotor = leftFront.getSelectedSensorPosition() * kDistancePerTick;
    double rightMotor = -rightFront.getSelectedSensorPosition() * kDistancePerTick;

    double averageMove = (leftMotor + rightMotor) / 2;

    return -averageMove;

  }

  // public void drive(double x, double y, double z) {
  //   drivetrain.driveCartesian(-x, y, z);
  // }
}