// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.pathconverter.PathConverter;
import frc.robot.subsystems.Autonomous;
import frc.robot.subsystems.RobotMechanisms;
import frc.swervesubsystem.SwerveDrive;

public class Robot extends TimedRobot {
  // Define objects and variables
  private Joystick leftStick = new Joystick(0);
  private Joystick rightStick = new Joystick(1);

  private double rightStickX;
  private double rightStickY;
  private double rightStickTwist;
  private double[] motorCurrents = new double[] {0, 0, 0, 0};
  private boolean manualArmRead = false;

  public NetworkTableInstance inst = NetworkTableInstance.getDefault();
  public NetworkTable driverStation;
  public NetworkTableEntry gyroAng;
  private ShuffleboardTab liveWindow = Shuffleboard.getTab("liveWindow");

  private SimpleWidget FFGain = liveWindow.add("FFGain", 0.000175);
  private SimpleWidget PGain = liveWindow.add("PGain", 0.00001);
  private SimpleWidget IGain = liveWindow.add("IGain", 0.0000004);
  private SimpleWidget DGain = liveWindow.add("DGain", 0.0);

  @Override
  public void robotInit() {
    // Initialize each class
    SwerveDrive.init();
    PathConverter.init();
    RobotMechanisms.init();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    
    /*
    RobotMechanisms.ArmExtendPIDController.setFF(FFGain.getEntry().getDouble(0));
    RobotMechanisms.ArmExtendPIDController.setP(PGain.getEntry().getDouble(0));
    RobotMechanisms.ArmExtendPIDController.setI(IGain.getEntry().getDouble(0));
    RobotMechanisms.ArmExtendPIDController.setD(DGain.getEntry().getDouble(0));
    */
    
    //SwerveDrive.updatePIDValues(FFGain.getEntry().getDouble(0), PGain.getEntry().getDouble(0),
    //    IGain.getEntry().getDouble(0), DGain.getEntry().getDouble(0), 8.0, 0.01, 0.01);
  }
 
  @Override
  public void teleopInit() {
    RobotMechanisms.extendLimelight();
  }

  @Override
  public void teleopPeriodic() {
    SwerveDrive.gyroRotation2d = SwerveDrive.gyro.getRotation2d().unaryMinus();

    // Assign stick inputs to variables, to prevent discrepancies
    rightStickX = rightStick.getX();
    rightStickY = rightStick.getY();
    rightStickTwist = rightStick.getRawAxis(3);

    // Create deadzones on the joysticks, to prevent stick drift
    if (Math.abs(rightStickX) < 0.1) {
      rightStickX = 0.0;
    }
    if (Math.abs(rightStickY) < 0.1) {
      rightStickY = 0.0;
    }
    if (Math.abs(rightStickTwist) < 0.2) {
      rightStickTwist = 0.0;
    }

    if (leftStick.getRawButton(2)) {
      //Tracking.centerOnPole();
    } else if (RobotMechanisms.desiredState != "Charge") {
      // Call swerveDrive() method, to do all the math and outputs for swerve drive
      SwerveDrive.calculateSpeedsAndAngles(Math.copySign(Math.pow(rightStickX, 2.0), rightStickX) * Constants.swerveMaxVelocity,
          (-Math.copySign(Math.pow(rightStickY, 2.0), rightStickY) * Constants.swerveMaxVelocity),
          (Math.copySign(Math.pow(rightStickTwist, 2.0), rightStickTwist) * Constants.swerveMaxVelocity),
          (1 - ((rightStick.getZ() + 1) / 2)), (1 - ((leftStick.getZ() + 1) / 2)));
      SwerveDrive.optimizeAndSetOutputs();
    }

    SmartDashboard.putNumber("Gyro", SwerveDrive.gyroRotation2d.getDegrees());

    motorCurrents[0] = SwerveDrive.frontLeft.drive.getOutputCurrent();
    motorCurrents[1] = SwerveDrive.frontRight.drive.getOutputCurrent();
    motorCurrents[2] = SwerveDrive.backLeft.drive.getOutputCurrent();
    motorCurrents[3] = SwerveDrive.backRight.drive.getOutputCurrent();
    SmartDashboard.putNumberArray("RobotDrive Motors", motorCurrents);
  
    if (rightStick.getRawButtonPressed(2) == true) {
      RobotMechanisms.reset();
    }
    if (leftStick.getRawButtonPressed(5)) {
      RobotMechanisms.desiredState = "Stowed";
      RobotMechanisms.armAngleMod = 0;
      RobotMechanisms.armExtendMod = 0;
    } else if (leftStick.getRawButtonPressed(2)) {
      RobotMechanisms.desiredState = "Grab";
      RobotMechanisms.armAngleMod = 0;
      RobotMechanisms.armExtendMod = 0;
    } else if (leftStick.getRawButtonPressed(6)) {
      RobotMechanisms.desiredState = "Place1";
      RobotMechanisms.armAngleMod = 0;
      RobotMechanisms.armExtendMod = 0;
    } else if (leftStick.getRawButtonPressed(4)) {
      RobotMechanisms.desiredState = "Place2";
      RobotMechanisms.armAngleMod = 0;
      RobotMechanisms.armExtendMod = 0;
    } else if (leftStick.getRawButtonPressed(3)) {
      RobotMechanisms.desiredState = "High";
      RobotMechanisms.armAngleMod = 0;
      RobotMechanisms.armExtendMod = 0;
    } // else if (rightStick.getRawButtonPressed(3)){
    //   RobotMechanisms.DesiredState = "Charge";
    //   RobotMechanisms.ArmAngleMod = 0;
    //   RobotMechanisms.ArmExtendMod = 0;
    // }
    if (rightStick.getRawButtonPressed(5)) {
      SwerveDrive.fieldOrientedSwerveEnabled = !SwerveDrive.fieldOrientedSwerveEnabled;
      System.out.println("Field oriented swerve state updated to " + SwerveDrive.fieldOrientedSwerveEnabled);
    }

    if (leftStick.getPOV() == 0 & !manualArmRead) {
      manualArmRead = true;
      RobotMechanisms.armAngleMod += 0.5;
    }
    if (leftStick.getPOV() == 180 & !manualArmRead) {
      manualArmRead = true;
      RobotMechanisms.armAngleMod -= 0.5;
    }
    if (leftStick.getPOV() == -1 & manualArmRead) {
      manualArmRead = false;
    }

    if (leftStick.getRawButtonPressed(1)) {
      RobotMechanisms.toggleGrabber();
    }
    if (rightStick.getRawButtonPressed(4)) {
      RobotMechanisms.extendLimelight();
    }
    if (rightStick.getRawButtonPressed(6)) {
      RobotMechanisms.retractLimelight();
    }

    RobotMechanisms.goToDesiredState();
  }

  //Autonomous right away
  @Override
  public void autonomousInit() {
    RobotMechanisms.extendLimelight();
    Autonomous.timer.stop();
    Autonomous.timer.reset();
  }

  //Autonomous repeat
  @Override
  public void autonomousPeriodic() {
    SwerveDrive.gyroRotation2d = SwerveDrive.gyro.getRotation2d().unaryMinus(); 
    Autonomous.runAutonomous();
  }
}