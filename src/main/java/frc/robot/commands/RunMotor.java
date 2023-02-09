// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.LessonSubsystem;

public class RunMotor extends CommandBase {
  /** Creates a new RunMotor. */
  private final LessonSubsystem lessonSubsystem;
  private final XboxController controller = RobotContainer.controller;

  public RunMotor(LessonSubsystem subsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    lessonSubsystem = subsystem;
    addRequirements(lessonSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (controller.getLeftY() > 0.25 || controller.getLeftY() < -0.25) {
      lessonSubsystem.drive((controller.getLeftY()) / 4.0);
    } else {
      lessonSubsystem.drive(0.0);
    }

    System.out.println(controller.getLeftY() + ", " + lessonSubsystem.getVelocityFeedback());

    if (controller.getBButtonPressed()) {
      lessonSubsystem.resetEncoders();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
