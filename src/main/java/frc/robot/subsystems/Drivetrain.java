// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.WPI_Pigeon2;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.kDrivetrain;
import frc.robot.Constants.kDrivetrain.kEncoder;
import frc.robot.Constants.kDrivetrain.kGyroscope;

public class Drivetrain extends Subsystem {

  // Motors
  private final CANSparkMax topLeft_mot;
  private final CANSparkMax topRight_mot;
  private final CANSparkMax bottomLeft_mot;
  private final CANSparkMax bottomRight_mot;

  // Encoders
  private final DutyCycleEncoder left_enc;
  private final DutyCycleEncoder right_enc;

  // Gyroscope
  private final WPI_Pigeon2 gyro;

  // Differential drive & odometry
  private final DifferentialDrive differential;
  private final DifferentialDriveOdometry odometry;

  // Shuffleboard & Field2D
  private final ShuffleboardTab field_tab;
  private final Field2d field2d;

  public Drivetrain() {

    // Motors
    topLeft_mot = new CANSparkMax(kDrivetrain.topLeftMotorID, MotorType.kBrushless);
    topLeft_mot.restoreFactoryDefaults();
    topLeft_mot.setIdleMode(IdleMode.kBrake);
    topLeft_mot.setInverted(true);
    topLeft_mot.setSmartCurrentLimit(kDrivetrain.kCurrentLimitAmps);
    topLeft_mot.burnFlash();

    topRight_mot = new CANSparkMax(kDrivetrain.topRightMotorID, MotorType.kBrushless);
    topRight_mot.restoreFactoryDefaults();
    topRight_mot.setIdleMode(IdleMode.kBrake);
    topRight_mot.setSmartCurrentLimit(kDrivetrain.kCurrentLimitAmps);
    topRight_mot.burnFlash();

    bottomLeft_mot = new CANSparkMax(kDrivetrain.bottomLeftMotorID, MotorType.kBrushless);
    bottomLeft_mot.restoreFactoryDefaults();
    bottomLeft_mot.setIdleMode(IdleMode.kBrake);
    bottomLeft_mot.setInverted(true);
    bottomLeft_mot.setSmartCurrentLimit(kDrivetrain.kCurrentLimitAmps);
    bottomLeft_mot.burnFlash();

    bottomRight_mot = new CANSparkMax(kDrivetrain.bottomRightMotorID, MotorType.kBrushless);
    bottomRight_mot.restoreFactoryDefaults();
    bottomRight_mot.setIdleMode(IdleMode.kBrake);
    bottomRight_mot.setSmartCurrentLimit(kDrivetrain.kCurrentLimitAmps);
    bottomRight_mot.burnFlash();

    bottomLeft_mot.follow(topLeft_mot);
    bottomRight_mot.follow(topRight_mot);

    // Encoders
    left_enc = new DutyCycleEncoder(kEncoder.leftEncoderDIOPort);
    left_enc.reset();
    left_enc.setDistancePerRotation(kEncoder.kEncoderDistancePerRotation);

    right_enc = new DutyCycleEncoder(kEncoder.rightEncoderDIOPort);
    right_enc.reset();
    right_enc.setDistancePerRotation(kEncoder.kEncoderDistancePerRotation);

    // Gyroscope
    gyro = new WPI_Pigeon2(kGyroscope.gyroscopeID);
    gyro.configMountPose(kGyroscope.kMountPoseForward, kGyroscope.kMountPoseUp);

    // Differential drive & odometry
    differential = new DifferentialDrive(topLeft_mot, topRight_mot);
    odometry = new DifferentialDriveOdometry(getGyroRotation(), getLeftDistanceTraveled(), getRightDistanceTraveled());

    // Shuffleboard & Field2D
    field_tab = Shuffleboard.getTab("Field2D");
    field2d = new Field2d();

    field_tab.add("Field", field2d);

  }

  // Default drive method that acceps trigger inputs
  public void doubleInputDrive(double forwardSpeed, double reverseSpeed, double rotation) {

    differential.arcadeDrive(forwardSpeed - reverseSpeed, rotation);

  }

  // Left encoder distance
  public double getLeftDistanceTraveled() {

    return left_enc.getDistance();

  }

  // Right encoder distance
  public double getRightDistanceTraveled() {

    return right_enc.getDistance();
    
  }

  // Gyroscope rotation
  public Rotation2d getGyroRotation() {

    return gyro.getRotation2d();
    
  }

  @Override
  public void periodic() {

    // Odometry & Field2D
    odometry.update(getGyroRotation(), getLeftDistanceTraveled(), getRightDistanceTraveled());
    field2d.setRobotPose(odometry.getPoseMeters());
    
  }

}
