// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.swervesubsystem.SwerveDrive;

public class Tracking extends SubsystemBase {
  public static NetworkTableInstance inst = NetworkTableInstance.getDefault();
  public static NetworkTable armLimelight = inst.getTable("limelight-arm");
  public static NetworkTableEntry armHasTarget = armLimelight.getEntry("tv");
  public static NetworkTableEntry armTargetOffsetH = armLimelight.getEntry("tx");
  public static NetworkTableEntry armTargetOffsetV = armLimelight.getEntry("ty");
  public static NetworkTableEntry armTargetArea = armLimelight.getEntry("ta");
  public static NetworkTableEntry armTargetSkew = armLimelight.getEntry("ts");
  public static NetworkTableEntry armPipeline = armLimelight.getEntry("pipeline");
  
  public static NetworkTable intakeLimelight = inst.getTable("limelight-intake");
  public static NetworkTableEntry intakeHasTarget = intakeLimelight.getEntry("tv");
  public static NetworkTableEntry intakeTargetOffsetH = intakeLimelight.getEntry("tx");
  public static NetworkTableEntry intakeTargetOffsetV = intakeLimelight.getEntry("ty");
  public static NetworkTableEntry intakeTargetArea = intakeLimelight.getEntry("ta");
  public static NetworkTableEntry intakeTargetSkew = intakeLimelight.getEntry("ts");
  public static NetworkTableEntry intakePipeline = intakeLimelight.getEntry("pipeline");
  private static Constraints PIDConstraints = new Constraints(1, .5);
  private static ProfiledPIDController PID = new ProfiledPIDController(1, 0, 0, PIDConstraints);

  public Tracking() {
  }

  /**
   * Center the robot on a pole.
   */
  public static void centerOnPole() {
    armPipeline.setValue(0);
    if (armTargetOffsetH.getDouble(0) >= 1) {
      SwerveDrive.calculateSpeedsAndAngles(-PID.calculate(armTargetOffsetH.getDouble(0), 0.0), 0.0, 0.0, .1, 0.0);
    }
    else {
      SwerveDrive.calculateSpeedsAndAngles(-PID.calculate(armTargetOffsetH.getDouble(0), 0.0), -PID.calculate(armTargetOffsetV.getDouble(0), 0.0), 0.0, .1, 0.0);
    }
    SwerveDrive.optimizeAndSetOutputs();
  }

  /**
   * Center the robot on a platform.
   */
  public static void centerOnPlatform() {
    armPipeline.setValue(1);
    if (armTargetOffsetH.getDouble(0) >= 1) {
      SwerveDrive.calculateSpeedsAndAngles(-PID.calculate(armTargetOffsetH.getDouble(0), 0.0), 0.0, 0.0, .1, 0.0);
    }
    else {
      SwerveDrive.calculateSpeedsAndAngles(-PID.calculate(armTargetOffsetH.getDouble(0), 0.0), -PID.calculate(armTargetOffsetV.getDouble(0), 0.0), 0.0, .1, 0.0);
    }
    SwerveDrive.optimizeAndSetOutputs();
  }

  /**
   * Center the robot on a cone.
   */
  public static void centerOnCone() {
    intakePipeline.setValue(0);
    if(Math.abs(intakeTargetOffsetH.getDouble(0)) >= 1) { //If centered on the cone with deadzone, move forward. Default prevents move
      SwerveDrive.calculateSpeedsAndAngles(-PID.calculate(intakeTargetOffsetH.getDouble(0), 0.0), 0, 0.0, .1, 0.0);
    } else {
      SwerveDrive.calculateSpeedsAndAngles(-PID.calculate(intakeTargetOffsetH.getDouble(0), 0.0), 1, 0.0, .1, 0.0);
    }
    SwerveDrive.optimizeAndSetOutputs();
  }
  
  /**
   * Center the robot on a cube.
   */
  public static void centerOnCube() {
    intakePipeline.setValue(1);
    if(Math.abs(intakeTargetOffsetH.getDouble(0)) >= 1) { //If centered on the cube with deadzone, move forward. Default prevents move
      SwerveDrive.calculateSpeedsAndAngles(-PID.calculate(intakeTargetOffsetH.getDouble(0), 0.0), 0, 0.0, .1, 0.0);
    } else {
      SwerveDrive.calculateSpeedsAndAngles(-PID.calculate(intakeTargetOffsetH.getDouble(0), 0.0), 1, 0.0, .1, 0.0);
    }
    SwerveDrive.optimizeAndSetOutputs();
  }
}
