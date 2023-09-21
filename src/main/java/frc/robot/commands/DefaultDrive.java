// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Drivetrain;

public class DefaultDrive extends Command {

  // Subsystems
  private final Drivetrain m_drivetrain;

  // Controller
  private final CommandXboxController m_controller;

  public DefaultDrive(Drivetrain drivetrain, CommandXboxController controller) {

    // Subsystems
    m_drivetrain = drivetrain;

    // Controller
    m_controller = controller;

    // Requirements
    addRequirements(m_drivetrain);

  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {

    // Default drive
    double forwardSpeed = m_controller.getRightTriggerAxis();
    double reverseSpeed = m_controller.getLeftTriggerAxis();
    double rotation = -m_controller.getLeftX();

    m_drivetrain.doubleInputDrive(forwardSpeed, reverseSpeed, rotation);

  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {

    return false;

  }

}
