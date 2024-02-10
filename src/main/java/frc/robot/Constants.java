// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  // Distance from center for wheels. Currently assumes rectangulare wheel arrangement
  public static double frontWheelPosition = 0.2604;
  public static double backWheelPosition = -0.2604;
  public static double rightWheelPosition = -0.2786;
  public static double leftWheelPosition = 0.2786;

  // CAN IDs for all swerve motor controllers
  public static int frontRightDriveCANID = 1;
  public static int frontRightSteerCANID = 5;

  public static int frontLeftDriveCANID = 2;
  public static int frontLeftSteerCANID = 6;

  public static int backLeftDriveCANID = 3;
  public static int backLeftSteerCANID = 7;

  public static int backRightDriveCANID = 4;
  public static int backRightSteerCANID = 8;


  // PID values for swerve PID loops
  public static double driveFeedForward = 0.000175;
  public static double driveProportional = 0.00001;
  public static double driveIntegral = 0.0000004;
  public static double driveDerivative = 0;

  public static double steerFeedForward = 0;
  public static double steerProportional = 8;
  public static double steerIntegral = 0.01;
  public static double steerDerivative = 0.01;

  // Charger PID values
  public static double chargeProportional = 1;
  public static double chargeIntegral = 1;
  public static double chargeDerivative = 1;

  // Number outputted by the steer encoders after 1 revolution
  public static int steerEncoderCountsPerRevolution = 1024;

  // Swerve drive maximum velocity in m/s
  public static double swerveMaxVelocity = 1;

  // Swerve drive maximum acceleration in m/s²
  public static double swerveMaxAcceleration = 1;
}