// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.PathPlanner;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.kOperatorConstants;
import frc.robot.Constants.kDrivetrain.kAuto;
import frc.robot.commands.DefaultDrive;
import frc.robot.commands.autoroutines.SplitPathA;
import frc.robot.subsystems.Drivetrain;

public class RobotContainer {

  // Controllers
  private final CommandXboxController primary_con;
  private final CommandXboxController secondary_con;

  // Subsystems
  private final Drivetrain sys_drivetrain;

  // Commands
  private final DefaultDrive sys_defaultDrive;

  // Auto routines
  private final SplitPathA sys_splitPathA;

  public RobotContainer() {

    // Controllers
    primary_con = new CommandXboxController(kOperatorConstants.kPrimaryControllerPort);
    secondary_con = new CommandXboxController(kOperatorConstants.kSecondaryControllerPort);

    // Subsystems
    sys_drivetrain = new Drivetrain();

    // Commands
    sys_defaultDrive = new DefaultDrive(sys_drivetrain, primary_con);

    // Auto routines
    sys_splitPathA = new SplitPathA(sys_drivetrain, PathPlanner.loadPathGroup("examplePath", kAuto.kMaxVelocity, kAuto.kMaxAccel));

    // Default commands
    sys_drivetrain.setDefaultCommand(sys_defaultDrive);

    configureBindings();
    
  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {

    return sys_splitPathA;

  }
}
