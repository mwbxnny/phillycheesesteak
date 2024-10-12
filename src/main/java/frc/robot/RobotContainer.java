// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Subsystems.Amp.Amp;
import frc.robot.Subsystems.Arm.Arm;
import frc.robot.Subsystems.Indexer.Indexer;
import frc.robot.Subsystems.Intake.Intake;
import frc.robot.Subsystems.Shooter.Shooter;

public class RobotContainer {
  private final CommandXboxController controller = new CommandXboxController(0);
  private final Amp amp = new Amp();
  private final Arm arm = new Arm();
  private final Indexer indexer = new Indexer();
  private final Intake intake = new Intake();
  private final Shooter shooter = new Shooter();
  
  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    controller.x().whileTrue(new RunCommand(() -> indexer.runIndexer(2)));
    controller.y().whileTrue(new RunCommand(() -> shooter.setVelocity(10, 1)));
    controller.a().onTrue(new InstantCommand(() -> intake.requestSetpoint(20)));
    controller.leftBumper().whileTrue(new RunCommand(() -> intake.requestIntakeVoltage(2)));
    controller.rightBumper().onTrue(new InstantCommand(() -> arm.requestSetpoint(20)));
    controller.b().whileTrue(new RunCommand(() -> amp.runAmp(2)));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
