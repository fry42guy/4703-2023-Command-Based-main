// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.pathconverter.PathConverter;
import frc.swervesubsystem.SwerveDrive;

public class Autonomous extends SubsystemBase {
  public static int autoStage = 0;
  private static int swerveControllerCommandIndex = 0;
  private static Boolean isScheduled = false;
  public static Timer timer = new Timer();

  public Autonomous() {}

  public static void runAutonomous() {
    if (autoStage <= PathConverter.autoOrder.size() - 1) {
      if (PathConverter.autoOrder.get(autoStage).equals("Move")) {
        if (!isScheduled) {
          System.out.println("Move");
          PathConverter.swerveControllerCommands.get(swerveControllerCommandIndex).andThen(() -> SwerveDrive.stop()).schedule();
          isScheduled = true;
        }
        if (PathConverter.swerveControllerCommands.get(swerveControllerCommandIndex).isFinished()) {
          autoStage++;
          swerveControllerCommandIndex++;
          isScheduled = false;
        }
      }
    }
    if (autoStage <= PathConverter.autoOrder.size() - 1) {
      if (PathConverter.autoOrder.get(autoStage).equals("Grab Cone")) {
        System.out.println("Grab Cone");
        RobotMechanisms.desiredState = "Grab";
        RobotMechanisms.openGrabber();
        if(RobotMechanisms.isAtDesiredState()) {
          if(timer.get() > 0) { //If we are grabbing the cone, we don't want to open grabber, nor keep moving
            Tracking.centerOnCone();
          }
          if(Math.abs(Tracking.intakeTargetOffsetV.getDouble(0)) <= 20) {//moved towards cone
            if(timer.get() <= 0.0) {
              RobotMechanisms.closeGrabber();
              timer.start();
            }
            if(timer.get() >= 0.1) { //waited for grabber to close
              RobotMechanisms.desiredState = "High";
              autoStage++;
              timer.stop();
              timer.reset();
            }
          }
        }
      }
    }
    if (autoStage <= PathConverter.autoOrder.size() - 1) {
      if (PathConverter.autoOrder.get(autoStage).equals("Grab Cube")) {
        System.out.println("Grab Cube");
        RobotMechanisms.desiredState = "Grab";
        RobotMechanisms.openGrabber();
        if(RobotMechanisms.isAtDesiredState()) {
          if(timer.get() > 0) { //If we are grabbing the cone, we don't want to open grabber, nor keep moving
            Tracking.centerOnCube();
          }
          if(Math.abs(Tracking.intakeTargetOffsetV.getDouble(0)) <= 20) {//moved towards cone
            if(timer.get() <= 0.0) {
              RobotMechanisms.closeGrabber();
              timer.start();
            }
            if(timer.get() >= 0.1) { //waited for grabber to close
              RobotMechanisms.desiredState = "High";
              autoStage++;
              timer.stop();
              timer.reset();
            }
          }
        }
      }
    }
    if (autoStage <= PathConverter.autoOrder.size() - 1) {
      if (PathConverter.autoOrder.get(autoStage).equals("Place Cone")) {
        System.out.println("Place Cone");
        RobotMechanisms.desiredState = "Place1";
        System.out.println("Arm Angle: " + RobotMechanisms.armAngle.getEncoder().getPosition());
        if(RobotMechanisms.isAtDesiredState()) {
          if(timer.get() <= 0.0) {
            RobotMechanisms.openGrabber();
            timer.start();
          }
          if(timer.get() >= 0.2) { //wait for grabber to open
            RobotMechanisms.desiredState = "Grab";
            autoStage++;
            timer.stop();
            timer.reset();
          }
          // Tracking.centerOnPole();
          // if(Math.abs(Tracking.ArmTargetOffsetH.getDouble(0)) <= 20) {
          //   if(Timer.get() <= 0.0) {
          //     RobotMechanisms.openGrabber();
          //     Timer.start();
          //   }
          //   if(Timer.get() >= 0.2) { //waited for grabber to open
          //     RobotMechanisms.DesiredState = "Stowed";
          //     AutoStage++;
          //     Timer.stop();
          //     Timer.reset();
          //   }
        }
      }
    }
    if (autoStage <= PathConverter.autoOrder.size() - 1) {
      if (PathConverter.autoOrder.get(autoStage).equals("Place Cube")) {
        System.out.println("Place Cube");
        RobotMechanisms.desiredState = "Place1";
        if(RobotMechanisms.isAtDesiredState()) {
          Tracking.centerOnPlatform();
          if(Math.abs(Tracking.armTargetOffsetH.getDouble(0)) <= 20) {
            RobotMechanisms.openGrabber();
            RobotMechanisms.desiredState = "High";
            autoStage++;
          }
        }
      }
    }
    if (autoStage <= PathConverter.autoOrder.size() - 1) {
      if (PathConverter.autoOrder.get(autoStage).equals("Charge")) {
        System.out.println("Charge");
        autoStage++;
      }
    }
    
    RobotMechanisms.goToDesiredState();
  }
}
