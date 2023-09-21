// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.sensors.Pigeon2.AxisDirection;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final class kOperatorConstants {

    public static final int kPrimaryControllerPort = 0;
    public static final int kSecondaryControllerPort = 1;

  }

  public static final class kDrivetrain {

    public static final int topLeftMotorID = 10;
    public static final int topRightMotorID = 11;
    public static final int bottomLeftMotorID = 12;
    public static final int bottomRightMotorID = 13;

    public static final class kEncoder {

      public static final int leftEncoderID = 20;
      public static final int rightEncoderID = 21;

      public static final int kCountsPerRotation = 4096;
      public static final double kSensorCoefficient = kWheel.kWheelCircumference / kCountsPerRotation;
      public static final String kUnitString = "m";

    }

    public static final class kGyroscope {

      public static final int gyroscopeID = 14;

      public static final AxisDirection kMountPoseForward = AxisDirection.PositiveY;
      public static final AxisDirection kMountPoseUp = AxisDirection.PositiveZ;

    }

    public static final class kWheel {

      public static final double kWheelDiameter = 0.15; // based off of Bishop; meters
      public static final double kWheelCircumference = Math.PI * kWheelDiameter; // meters

    }

    public static final class kSimulation {

      public static final double kGearing = 5.0;
      public static final double kJKgMeterSq = 3;
      public static final double kMass = 50; // based off of Bishop
      public static final double kWheelRadius = kWheel.kWheelDiameter / 2;
      public static final double kTrackWidth = 0.6; // based off of Bishop
      public static final double kStandardDevs = 0.0; // set to null
      
    }

    public static final int kCurrentLimitAmps = 30;

  }

}
