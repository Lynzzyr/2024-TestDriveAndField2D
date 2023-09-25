// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPRamseteCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.kDrivetrain;
import frc.robot.Constants.kDrivetrain.kAuto;
import frc.robot.Constants.kDrivetrain.kFeedforward;
import frc.robot.subsystems.Drivetrain;

public class AutoFollowPath extends SequentialCommandGroup {

  public AutoFollowPath(Drivetrain drivetrain, PathPlannerTrajectory trajectory) {

    super(

      new PPRamseteCommand(

        trajectory,
        drivetrain::getPose2D,
        new RamseteController(kAuto.kRamseteB, kAuto.kRamseteZeta),
        new SimpleMotorFeedforward(kFeedforward.ksVolts, kFeedforward.kvVolts, kFeedforward.kaVolts),
        kDrivetrain.kDriveKinematics,
        drivetrain::getWheelSpeeds,
        new PIDController(kAuto.kPDrive, 0, 0),
        new PIDController(kAuto.kPDrive, 0, 0),
        drivetrain::setMotorVolts,
        drivetrain

      )

    );

  }
}
