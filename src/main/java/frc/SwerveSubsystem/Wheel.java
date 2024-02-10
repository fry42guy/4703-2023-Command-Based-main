// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.swervesubsystem;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxRelativeEncoder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/** 
* Defines objects and variables for each wheel module.
*/
public class Wheel extends SubsystemBase {
  // Define objects and variables
  public CANSparkMax drive;
  private RelativeEncoder driveEncoder;
  private SparkMaxPIDController drivePIDController;
  TalonSRX steer;
  private double steerAngRot = 0.0;
  private double steerAngRad = 0.0;
  private double steerFullRot = 0.0;
  Translation2d location;
  SwerveModuleState moduleState;
  private boolean isInput = false;

  /**
  * Class constructor for the Wheel class, initializes all variables, objects, and methods for the created Wheel object.
  * 
  * @param moduleLocationX
  *     X position of the wheel module relative to the center of the robot in meters.
  * @param moduleLocationY
  *     Y position of the wheel module relative to the center of the robot in meters.
  */
  public Wheel(double moduleLocationX, double moduleLocationY) {
    location = new Translation2d(moduleLocationX, moduleLocationY);
  }

  /**
	 * Define what the objects "steerEncoder" and "steerPIDController" refer to, and initialize them.
	*/
  public void initEncodersAndPIDControllers() {
    // Make the steer motors prevent movement when there isn't an input
    steer.setNeutralMode(NeutralMode.Brake);

    steer.configClosedloopRamp(0);

    // Tell the steer motor controller that an encoder exists, and what kind it is
    steer.configSelectedFeedbackSensor(FeedbackDevice.Analog);

    // Define what encoder the object "driveEncoder" refers to
    driveEncoder = drive.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42);
    
    // Zero relative encoders, just in case
    driveEncoder.setPosition(0);
    
    // Define what PID controller the object "drivePIDController" refers to
    drivePIDController = drive.getPIDController();
    
    // Tell the PID controller what encoder to use
    drivePIDController.setFeedbackDevice(driveEncoder);

    // Set max and min values to be sent to the motors by the PID controllers. Likely shouldn't be changed.
    steer.configClosedLoopPeakOutput(0, 1);
    drivePIDController.setOutputRange(-1, 1);
  }

  /**
   * Set the P, I, and D values for the PID controllers.
   * 
   * @param DFF
   *     Drive Feed-Forward value
   * @param DP
	 *     Drive Proportional value
   * @param DI
	 *     Drive Integral value
   * @param DD
	 *     Drive Derivative value
   * @param SFF
	 *     Steer Feed Forward value
   * @param SP
	 *     Steer Proportional value
   * @param SI
	 *     Steer Integral value
   * @param SD
	 *     Steer Derivative value
   */
  public void updatePIDValues(double DFF, double DP, double DI, double DD, double SFF, double SP, double SI, double SD) {
    drivePIDController.setFF(DFF);
    drivePIDController.setP(DP);
    drivePIDController.setI(DI);
    drivePIDController.setD(DD);
    steer.config_kF(0, SFF);
    steer.config_kP(0, SP);
    steer.config_kI(0, SI);
    steer.config_kD(0, SD);
  }

  /**
   * Do math and set multiple variables required to make the absolute encoders function properly.
   */
  public void setEncoderVariables() {
    // Store position of the steer encoder to prevent discrepancies
    steerAngRot = (steer.getSelectedSensorPosition() / Constants.steerEncoderCountsPerRevolution);

    // Get the number of full rotations the wheel has gone through
    steerFullRot = Math.floor(steerAngRot);

    // Invert the angle, so wpilib's math has a good input
    steerAngRad = ((2 * Math.PI) - ((2 * Math.PI) * (steerAngRot - steerFullRot)));
  }

  /**
   * Do math for the swerve drive that each wheel has to call, and then output the desired angle and speed to the wheel.
   */
  public void optimizeAndCalculateVariables() {
    // Optimize rotation positions, so the wheels don't turn 180 degrees rather than just spinning the drive motor backwards
    // Determine if the distance between the desired angle and the current angle is less than or equal to 90
    // This is to determine whether the drive motors should be driven forward or backward.
    // If the difference between the desired and current positions is less than or equal to 90 degrees, then...
    if ((Math.abs(moduleState.angle.getRadians() - steerAngRad) <= (Math.PI / 2)) || (Math.abs(moduleState.angle.getRadians() - steerAngRad) >= ((3 * Math.PI) / 2))) {
      // If the wheel would have to cross into a new rotation to travel the shortest distance to the desired angle, then...
      if (Math.abs(moduleState.angle.getRadians() - steerAngRad) >= ((3 * Math.PI) / 2)) {
        // If the difference is positive, then...
        if (moduleState.angle.getRadians() > steerAngRad) {
          // Subtract 2pi from the desired angle to show the PID controller later that the shortest distance is to cross 0
          moduleState = new SwerveModuleState(moduleState.speedMetersPerSecond, new Rotation2d(moduleState.angle.getRadians() - (2 * Math.PI)));
        }
        // If the difference is negative, then...
        else {
          // Add 2pi to the desired angle to show the PID controller later that the shortest distance is to cross 2pi
          moduleState = new SwerveModuleState(moduleState.speedMetersPerSecond, new Rotation2d(moduleState.angle.getRadians() + (2 * Math.PI)));
        }
      }

    }
    // If the difference between the desired and current positions is greater than 90 degrees, then...
    else {
      // If the difference is positive, then...
      if (moduleState.angle.getRadians() > steerAngRad) {
        // Invert the drive motor output, and flip the desired angle by subtracting pi
        moduleState = new SwerveModuleState(-moduleState.speedMetersPerSecond, new Rotation2d(moduleState.angle.getRadians() - Math.PI));
      }
      // If the difference is negative, then...
      else {
        // Invert the drive motor output, and flip the desired angle by adding pi
        moduleState = new SwerveModuleState(-moduleState.speedMetersPerSecond, new Rotation2d(moduleState.angle.getRadians() + Math.PI));
      }
    }

    // Re-invert the angle, so the PID controller has a good input
    moduleState = new SwerveModuleState(moduleState.speedMetersPerSecond, new Rotation2d(((2 * Math.PI) - moduleState.angle.getRadians()) + (steerFullRot * (2 * Math.PI))));

    // Check if any input is being sent, to prevent wheels from rotating to 0 when no input. 
    if (moduleState.speedMetersPerSecond != 0) {
      isInput = true;
    }
    else {
      isInput = false;
    }
  }

  /**
   * Set the output speed and angle of the wheels.
   */
  public void setOutputs() {
    // Tell the steer motor to turn the wheel to the correct position
    // An issue is created by ramping which this if statement solves, I will explain the root of the problem, and the solution here:
    // If all inputs for robot speeds are 0, the angle for the wheel will default to 0
    // This causes a problem because the drive wheel speed does not instantly go to zero, causing the robot's direction to change
    // This if statement fixes this issue by only changing the angle of the wheel if and only if any of the desired robot speeds are greater than 0
    if (isInput == true) {
      steer.set(ControlMode.Position, (moduleState.angle.getDegrees() / 360.0) * Constants.steerEncoderCountsPerRevolution);
    }

    //steer.set(ControlMode.Position, 0);

    // Tell the drive motor to drive the wheels at the correct speed
    drivePIDController.setReference((((moduleState.speedMetersPerSecond / ((4 / 39.37) * Math.PI)) * 60) / .15), ControlType.kVelocity);
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition((((driveEncoder.getPosition()) * .15) * ((4 / 39.37) * Math.PI)), new Rotation2d(steerAngRad));
  }
}