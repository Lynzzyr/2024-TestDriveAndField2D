// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.BasePigeonSimCollection;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.CANCoderSimCollection;
import com.ctre.phoenix.sensors.WPI_CANCoder;
import com.ctre.phoenix.sensors.WPI_Pigeon2;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.kDrivetrain;
import frc.robot.Constants.kDrivetrain.kEncoder;
import frc.robot.Constants.kDrivetrain.kGyroscope;
import frc.robot.Constants.kDrivetrain.kSimulation;
import frc.robot.Constants.kDrivetrain.kWheel;

public class Drivetrain extends Subsystem {

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

  // Simulation
  private final CANCoderSimCollection simLeft_enc;
  private final CANCoderSimCollection simRight_enc;

  private final BasePigeonSimCollection simGyro;

  private final DifferentialDrivetrainSim simDifferential;

  // Shuffleboard & Field2D
  private final ShuffleboardTab field_tab;
  private final Field2d field2d;

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

    // Simulation
    simLeft_enc = left_enc.getSimCollection();
    simRight_enc = right_enc.getSimCollection();

    simGyro = gyro.getSimCollection();

    simDifferential = new DifferentialDrivetrainSim(

      DCMotor.getNEO(2),
      kSimulation.kGearing,
      kSimulation.kJKgMeterSq,
      kSimulation.kMass,
      kSimulation.kWheelRadius,
      kSimulation.kTrackWidth,
      null

    );

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

    // Odometry & Field2D
    odometry.update(getGyroRotation(), getLeftDistanceTraveled(), getRightDistanceTraveled());
    field2d.setRobotPose(odometry.getPoseMeters());
    
  }

  @Override
  public void simulationPeriodic() {

    simDifferential.setInputs(topLeft_mot.get(), -topRight_mot.get());
    simDifferential.update(0.02);

    simLeft_enc.setRawPosition((int)(simDifferential.getLeftPositionMeters() * (kEncoder.kCountsPerRotation / kWheel.kWheelCircumference)));
    simLeft_enc.setVelocity((int)((simDifferential.getLeftVelocityMetersPerSecond() * (kEncoder.kCountsPerRotation / kWheel.kWheelCircumference)) / 10));
    simRight_enc.setRawPosition((int)(simDifferential.getRightPositionMeters() * (kEncoder.kCountsPerRotation / kWheel.kWheelCircumference)));
    simRight_enc.setVelocity((int)((simDifferential.getRightVelocityMetersPerSecond() * (kEncoder.kCountsPerRotation / kWheel.kWheelCircumference)) / 10));
    simGyro.setRawHeading(-simDifferential.getHeading().getDegrees());

  }

}
