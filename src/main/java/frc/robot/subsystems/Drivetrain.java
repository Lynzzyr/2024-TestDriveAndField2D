// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.WPI_CANCoder;
import com.ctre.phoenix.sensors.WPI_Pigeon2;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.kDrivetrain;
import frc.robot.Constants.kDrivetrain.kEncoder;
import frc.robot.Constants.kDrivetrain.kGyroscope;

public class Drivetrain extends SubsystemBase {

  // Motors
  private final CANSparkMax topLeft_mot;
  private final CANSparkMax topRight_mot;
  private final CANSparkMax bottomLeft_mot;
  private final CANSparkMax bottomRight_mot;

  // Encoders
  private final CANCoderConfiguration encoderConfig;
  private final WPI_CANCoder left_enc;
  private final WPI_CANCoder right_enc;

  // Gyroscope
  private final WPI_Pigeon2 gyro;

  // Differential drive & odometry
  private final DifferentialDrive differential;
  private final DifferentialDriveOdometry odometry;

  public Drivetrain() {

    // Motors
    topLeft_mot = new CANSparkMax(kDrivetrain.topLeftMotorID, MotorType.kBrushless);
    topLeft_mot.restoreFactoryDefaults();
    topLeft_mot.setIdleMode(IdleMode.kBrake);
    topLeft_mot.setSmartCurrentLimit(kDrivetrain.kCurrentLimitAmps);
    topLeft_mot.burnFlash();

    topRight_mot = new CANSparkMax(kDrivetrain.topRightMotorID, MotorType.kBrushless);
    topRight_mot.restoreFactoryDefaults();
    topRight_mot.setIdleMode(IdleMode.kBrake);
    topRight_mot.setInverted(true);
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
    bottomRight_mot.setInverted(true);
    bottomRight_mot.setSmartCurrentLimit(kDrivetrain.kCurrentLimitAmps);
    bottomRight_mot.burnFlash();

    bottomLeft_mot.follow(topLeft_mot);
    bottomRight_mot.follow(topRight_mot);

    // Encoders
    encoderConfig = new CANCoderConfiguration();
    encoderConfig.sensorCoefficient = kEncoder.kSensorCoefficient;
    encoderConfig.unitString = kEncoder.kUnitString;

    left_enc = new WPI_CANCoder(kEncoder.leftEncoderID);
    left_enc.configFactoryDefault();
    left_enc.configAllSettings(encoderConfig);

    right_enc = new WPI_CANCoder(kEncoder.rightEncoderID);
    right_enc.configFactoryDefault();
    right_enc.configAllSettings(encoderConfig);

    // Gyroscope
    gyro = new WPI_Pigeon2(kGyroscope.gyroscopeID);
    gyro.configMountPose(kGyroscope.kMountPoseForward, kGyroscope.kMountPoseUp);

    // Differential drive & odometry
    differential = new DifferentialDrive(topLeft_mot, topRight_mot);
    odometry = new DifferentialDriveOdometry(getGyroRotation(), getLeftDistanceTraveled(), getRightDistanceTraveled());

  }

  // Default drive method that acceps trigger inputs
  public void doubleInputDrive(double forwardSpeed, double reverseSpeed, double rotation) {

    differential.arcadeDrive(forwardSpeed - reverseSpeed, rotation);

  }

  // Left encoder distance
  public double getLeftDistanceTraveled() {

    return left_enc.getPosition();

  }

  // Right encoder distance
  public double getRightDistanceTraveled() {

    return right_enc.getPosition();
    
  }

  // Gyroscope rotation
  public Rotation2d getGyroRotation() {

    return gyro.getRotation2d();
    
  }

  @Override
  public void periodic() {

    // Odometry
    odometry.update(getGyroRotation(), getLeftDistanceTraveled(), getRightDistanceTraveled());
    
  }

}
