// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Subsystems.Amp.Amp;
import frc.robot.Subsystems.Arm.Arm;
import frc.robot.Subsystems.Indexer.Indexer;
import frc.robot.Subsystems.Intake.Intake;
import frc.robot.Subsystems.Shooter.Shooter;

public class RobotContainer {
  public RobotContainer() {
    final CommandXboxController controller = new CommandXboxController(0);
    final Amp amp = new Amp();
    final Arm arm = new Arm();
    final Indexer indexer = new Indexer();
    final Intake intake = new Intake();
    final Shooter shooter = new Shooter();
    configureBindings();
  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
