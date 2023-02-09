// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class LessonSubsystem extends SubsystemBase {
  /** Creates a new LessonSubsystem. */
  public static final WPI_TalonFX lessonMotor = RobotMap.lessonMotor;
  private static final double IN_TO_M = .0254;
  // public static final int MOTOR_ENCODER_COUNTS_PER_REV = 2048; //4096 for CTRE Mag Encoders, 2048 for the Falcons
  private static final double DIAMETER_INCHES = 6.0; // wheel diameter
  private static final double WHEEL_DIAMETER = DIAMETER_INCHES * IN_TO_M; // in meters
  private static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;
  public static final double GEAR_RATIO = 10.71;
  public static final double TICKS_PER_METER = (2048 * GEAR_RATIO) / (WHEEL_CIRCUMFERENCE);

  public LessonSubsystem() {
    lessonMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 10);
    lessonMotor.configNominalOutputForward(0, 10);
    lessonMotor.configNominalOutputReverse(0, 10);
    lessonMotor.configPeakOutputForward(1, 10);
    lessonMotor.configPeakOutputReverse(-1, 10);
    lessonMotor.configNeutralDeadband(0.001, 10);
    lessonMotor.configAllowableClosedloopError(0, 0, 10);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void drive(double command) {

    double driveSpeedPer100MS = (TICKS_PER_METER * (1.0/1000.0) * 100.0); //tick second

    lessonMotor.set(TalonFXControlMode.Velocity, driveSpeedPer100MS*command);
  }

  public static double getPositionFeedback() {
    return lessonMotor.getSelectedSensorPosition();
  }

  public static double getVelocityFeedback() {
    return lessonMotor.getSelectedSensorVelocity();
  }
  
  public static void resetEncoders() {
    lessonMotor.setSelectedSensorPosition(0.0);
  }
}
