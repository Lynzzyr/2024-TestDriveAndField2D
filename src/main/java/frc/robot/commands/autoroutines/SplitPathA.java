// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autoroutines;

import java.util.List;

import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AutoFollowPath;
import frc.robot.subsystems.Drivetrain;


public class SplitPathA extends SequentialCommandGroup {

  public SplitPathA(Drivetrain drivetrain, List<PathPlannerTrajectory> trajectoryGroup) {

    addCommands(

      Commands.runOnce(() -> drivetrain.resetOdometry(trajectoryGroup.get(0).getInitialPose())),

      new AutoFollowPath(drivetrain, trajectoryGroup.get(0)),

      Commands.waitSeconds(2),

      new AutoFollowPath(drivetrain, trajectoryGroup.get(1))

    );

  }
}
