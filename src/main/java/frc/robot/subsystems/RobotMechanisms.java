// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxRelativeEncoder;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.swervesubsystem.SwerveDrive;

public class RobotMechanisms extends SubsystemBase {
  static CANSparkMax armAngle = new CANSparkMax(10, MotorType.kBrushless);
  private static SparkMaxPIDController armAnglePIDController = armAngle.getPIDController();
  private static CANSparkMax armExtend = new CANSparkMax(11, MotorType.kBrushless);
  private static SparkMaxPIDController armExtendPIDController = armExtend.getPIDController();

  private static Constraints chargeConstraints = new Constraints(Constants.swerveMaxVelocity, Constants.swerveMaxAcceleration);
  private static ProfiledPIDController chargePIDController = new ProfiledPIDController(1, 0, 0, chargeConstraints);

  private static Compressor pump = new Compressor(0, PneumaticsModuleType.CTREPCM);
  private static DoubleSolenoid grabber = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 2, 3);
  private static DoubleSolenoid limelightPiston = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 1);

  private static DigitalInput limitSwitch = new DigitalInput(0);
  private static boolean hasBeenZeroed = false;

  public static String desiredState = "";
  private static double angleToAdjust;
  public static double armAngleMod = 0;
  public static double armExtendMod = 0;

  public RobotMechanisms() {
  }

  public static void init() {
    armAnglePIDController.setFeedbackDevice(armAngle.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42));
    armAnglePIDController.setOutputRange(-1, 1);
    armAnglePIDController.setFF(0);
    armAnglePIDController.setP(.06);
    armAnglePIDController.setI(0);
    armAnglePIDController.setD(0);
    armAnglePIDController.setSmartMotionMinOutputVelocity(0, 0);
    armAnglePIDController.setSmartMotionMaxVelocity(120, 0);
    armAnglePIDController.setSmartMotionMaxAccel(50, 0);

    armExtendPIDController.setFeedbackDevice(armExtend.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42));
    armExtendPIDController.setOutputRange(-1, 1);
    armExtendPIDController.setFF(0);
    armExtendPIDController.setP(.06);
    armExtendPIDController.setI(0);
    armExtendPIDController.setD(0);
    armExtendPIDController.setSmartMotionMinOutputVelocity(0, 0);
    armExtendPIDController.setSmartMotionMaxVelocity(120, 0);

    pump.enableDigital();
    grabber.set(Value.kForward);
    limelightPiston.set(Value.kForward);
  }

  /**
   * Make the robot go to the desired state.
   */
  public static void goToDesiredState() {   
    if (hasBeenZeroed) {
      if (desiredState.equals("Stowed")) {
        closeGrabber();
        if (Math.abs(armExtend.getEncoder().getPosition()) > .6) {
          armExtendPIDController.setReference(0, ControlType.kPosition);
        } else {
          armAnglePIDController.setReference(0, ControlType.kPosition);
          armExtendPIDController.setReference(0, ControlType.kPosition);
        }
      }

      if (desiredState.equals("Grab")) {
        if (armAngle.getEncoder().getPosition() < -7 & armExtend.getEncoder().getPosition() < -43) {
          armExtendPIDController.setReference(-52, ControlType.kPosition);
        } else if (armAngle.getEncoder().getPosition() > -3 & armExtend.getEncoder().getPosition() < -.6) {
          armExtendPIDController.setReference(0, ControlType.kPosition);
        } else if (armAngle.getEncoder().getPosition() > -4) {
          armAnglePIDController.setReference(-6.5 - armAngleMod, ControlType.kPosition);
        } else {
          armAnglePIDController.setReference(-6.5 - armAngleMod, ControlType.kPosition);
          armExtendPIDController.setReference(-52, ControlType.kPosition);
        }
      }

      if (desiredState.equals("Place1")) {
        System.out.println("Attempting Place1 Command");
        if (armAngle.getEncoder().getPosition() > -6) {
          armAnglePIDController.setReference(-19 - armAngleMod, ControlType.kPosition);
          armExtendPIDController.setReference(0, ControlType.kPosition);
        } else {
          armAnglePIDController.setReference(-19 - armAngleMod, ControlType.kPosition);
          armExtendPIDController.setReference(-37, ControlType.kPosition);
        }
      }

      if (desiredState.equals("Place2")) {
        if (armAngle.getEncoder().getPosition() > -6) {
          armAnglePIDController.setReference(-23 - armAngleMod, ControlType.kPosition);
          armExtendPIDController.setReference(0, ControlType.kPosition);
        } else {
          armAnglePIDController.setReference(-23 - armAngleMod, ControlType.kPosition);
          armExtendPIDController.setReference(-85, ControlType.kPosition);
        }
      }
      if (desiredState.equals("High")) {
        if (armAngle.getEncoder().getPosition() > -6) {
          armAnglePIDController.setReference(-23 - armAngleMod, ControlType.kPosition);
          armExtendPIDController.setReference(0, ControlType.kPosition);
        } else {
          armAnglePIDController.setReference(-23 - armAngleMod, ControlType.kPosition);
          armExtendPIDController.setReference(0, ControlType.kPosition);
        }
      }
      if (desiredState.equals("charge")) {
        if (Math.abs(SwerveDrive.gyro.getRoll()) > Math.abs(SwerveDrive.gyro.getPitch())) {
          angleToAdjust = SwerveDrive.gyro.getRoll();
        } else {
          angleToAdjust = SwerveDrive.gyro.getPitch();
        }
        System.out.println(angleToAdjust);
        SwerveDrive.calculateSpeedsAndAngles(0, chargePIDController.calculate(angleToAdjust, 0)/300, 0, 1, 1);
        SwerveDrive.optimizeAndSetOutputs();
        System.out.println("Output:" + chargePIDController.calculate(angleToAdjust * 20, 0));
      }
    }
    else {
      if (!limitSwitch.get()) {
        hasBeenZeroed = true;
        armExtend.getEncoder().setPosition(0);
        armExtendPIDController.setReference(0, ControlType.kPosition);
      }
      else {
        armExtend.set(.2);
      }
    } 
  }

  /**
   * Check if the robot is at the desired state.
   * 
   * @return A boolean value that is true if the robot is at the desired state, and is false if not.
   */
  public static boolean isAtDesiredState() {
    boolean ret = false;
    if (desiredState == "Stowed" & Math.abs(armExtend.getEncoder().getPosition()) <= .6 & Math.abs(armExtend.getEncoder().getPosition()) <= 1) {
      ret = true;
    }
    if (desiredState == "Grab" & Math.abs(Math.abs(armAngle.getEncoder().getPosition()) - 6.5) <= 1
        & Math.abs(Math.abs(armExtend.getEncoder().getPosition()) - 41) <= 1) {
      ret = true;
    }
    if (desiredState == "Place1" & Math.abs(Math.abs(armAngle.getEncoder().getPosition()) - 17) <= 1
        & Math.abs(Math.abs(armExtend.getEncoder().getPosition()) - 37) <= 1) {
      ret = true;
    }
    if (desiredState == "Place2" & Math.abs(Math.abs(armAngle.getEncoder().getPosition()) - 21) <= 1
        & Math.abs(Math.abs(armExtend.getEncoder().getPosition()) - 85) <= 1) {
      ret = true;
    }
    if (desiredState == "High" & Math.abs(Math.abs(armAngle.getEncoder().getPosition()) - 21) <= 1
        & Math.abs(Math.abs(armExtend.getEncoder().getPosition())) <= 1) {
      ret = true;
    }
    return ret;
  }

  /**
   * Command the robot to toggle the grabber.
   */
  public static void toggleGrabber() {
    grabber.toggle();
  }

  /**
   * Command the robot to close the grabber.
   */
  public static void closeGrabber() {
    if (grabber.get() != Value.kForward) {
      grabber.set(Value.kForward);
    }
  }

  /**
   * Command the robot to open the grabber.
   */
  public static void openGrabber() {
    if (grabber.get() != Value.kReverse) {
      grabber.set(Value.kReverse);
    }
  }

  /**
   * Command the robot to extend the Limelight piston.
   */
  public static void extendLimelight() {
    limelightPiston.set(Value.kReverse);
  }

  /**
   * Command the robot to retract the Limelight piston.
   */
  public static void retractLimelight() {
    limelightPiston.set(Value.kForward);
  }

  /**
   * Reset the robot's gyroscope.
   */
  public static void reset() {
    SwerveDrive.gyro.reset();
  }
}
