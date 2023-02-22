// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.LessonSubsystem;

public class RunMotor extends CommandBase {
  /** Creates a new RunMotor. */
  private final LessonSubsystem lessonSubsystem;
  private final XboxController controller = RobotContainer.controller;

  public static double command = 0.0;
  public static double time = 0.0;
  public static double A = 200; //ticks per 100 ms
  public static double frequency = 0.1; //hertz
  public static boolean sign = true;

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
    //command = A * Math.sin(2.0*Math.PI*frequency*time);
    time = time + 0.02;
    if (time > 4.0) {
      time = 0.0;
      if (!sign) {
        sign = true;
      } else {
        sign = false;
      }
    }

    if (sign) {
      command = 400.0;
    } else {
      command = -400.0;
    }

    lessonSubsystem.drive(command);
    System.out.println("Time: " + time + ", command: " + command + ", feedback: " + lessonSubsystem.getVelocityFeedback());
    SmartDashboard.putNumber("Command", command);
    SmartDashboard.putNumber("Velocity Error", lessonSubsystem.getVelocityError(command));
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
