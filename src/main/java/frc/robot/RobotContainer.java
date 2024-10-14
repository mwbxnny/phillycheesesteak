// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Subsystems.Amp.AmpIOReal;
import frc.robot.Subsystems.Arm.ArmIOReal;
import frc.robot.Subsystems.Indexer.IndexerIOReal;
import frc.robot.Subsystems.Intake.IntakeIOReal;
import frc.robot.Subsystems.Shooter.ShooterIOReal;

public class RobotContainer {
  private final CommandXboxController controller = new CommandXboxController(0);
  private final AmpIOReal amp = new AmpIOReal();
  private final ArmIOReal arm = new ArmIOReal();
  private final IndexerIOReal indexer = new IndexerIOReal();
  private final IntakeIOReal intake = new IntakeIOReal();
  private final ShooterIOReal shooter = new ShooterIOReal();
  
  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    controller.x().whileTrue(new RunCommand(() -> indexer.runIndexer(2)));
    controller.y().whileTrue(new RunCommand(() -> shooter.setVelocity(10, 1)));
    controller.a().onTrue(new InstantCommand(() -> intake.setPivotPosition(20)));
    controller.leftBumper().whileTrue(new RunCommand(() -> intake.setIntakeVoltage(2)));
    controller.rightBumper().onTrue(new InstantCommand(() -> arm.setSetpoint(20)));
    controller.b().whileTrue(new RunCommand(() -> amp.runAmp(2)));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
