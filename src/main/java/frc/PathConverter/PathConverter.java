// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.pathconverter;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.constraint.SwerveDriveKinematicsConstraint;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.swervesubsystem.SwerveDrive;
import java.io.File;
import java.io.FileNotFoundException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Scanner;

public class PathConverter extends SubsystemBase {
  private static TrajectoryLoader trajectoryLoader = new TrajectoryLoader();
  private static List<SendableChooser<String>> autoChoosers = Arrays.asList(new SendableChooser<String>(), new SendableChooser<String>());
  private static int autoChooserListID;
  private static String[] autoNames = Filesystem.getDeployDirectory().toPath().resolve("output/paths").toFile().list();
  private static String autoFile;
  private static File trajFile;
  private static SwerveDriveKinematicsConstraint swerveDriveMaxSpeed = new SwerveDriveKinematicsConstraint(SwerveDrive.kinematics, Constants.swerveMaxVelocity);
  private static Constraints PIDConstraints = new Constraints(Constants.swerveMaxVelocity, Constants.swerveMaxAcceleration);
  private static TrajectoryConfig trajConfig = new TrajectoryConfig(Constants.swerveMaxVelocity, Constants.swerveMaxAcceleration).setKinematics(SwerveDrive.kinematics).addConstraint(swerveDriveMaxSpeed);
  private static Trajectory trajectory;
  private static Scanner autoReader;
  private static List<String> lines = new ArrayList<String>();
  private static List<String> currentLine = new ArrayList<String>();
  private static List<String> fileOrder = new ArrayList<String>();
  public static List<String> autoOrder = new ArrayList<String>();
  private static List<Translation2d> translation2ds = new ArrayList<Translation2d>();
  private static List<Rotation2d> rotation2ds = new ArrayList<Rotation2d>();
  private static List<Pose2d> pose2ds = new ArrayList<Pose2d>();
  private static List<Translation2d> middlePoints = new ArrayList<Translation2d>();
  public static List<SwerveControllerCommand> swerveControllerCommands = new ArrayList<SwerveControllerCommand>();
  private static int startIndex;

  public static class TrajectoryLoader extends CommandBase {
    @Override
    public boolean runsWhenDisabled() {
      return true;
    }

    @Override
    public void initialize() {
      autoFile = "None";
    }
  
    @Override
    public void execute() {
      if (autoFile != autoChoosers.get(autoChooserListID).getSelected()) {
        autoFile = autoChoosers.get(autoChooserListID).getSelected();
        System.out.println(autoFile);
        try {
          initTrajectory();
        } catch (FileNotFoundException e) {
          System.out.println("AUTO NOT FOUND");
        }
      }

      if ((DriverStation.getAlliance() == DriverStation.Alliance.Red & autoChooserListID != 0)
          || (DriverStation.getAlliance() == DriverStation.Alliance.Blue & autoChooserListID != 1)) {
        if (DriverStation.getAlliance() == DriverStation.Alliance.Red) {
          autoChooserListID = 0;
        }
        if (DriverStation.getAlliance() == DriverStation.Alliance.Blue) {
          autoChooserListID = 1;
        }
        SmartDashboard.putData("autoChooser", autoChoosers.get(autoChooserListID));
      }
    }
  }

  public PathConverter() {
  }

  public static void init() {
    for (Integer i = 0; i <= autoNames.length - 1; i++) {
      // Create auto options for each alliance
      if (autoNames[i].toString().contains("Red")) {
        autoChoosers.get(0).addOption(autoNames[i], autoNames[i]);
      }
      else if (autoNames[i].toString().contains("Blue")) {
        autoChoosers.get(1).addOption(autoNames[i], autoNames[i]);
      }
    }
    if (DriverStation.getAlliance() == DriverStation.Alliance.Red) {
      autoChooserListID = 0;
    }
    else {
      autoChooserListID = 1;
    }
    autoChoosers.get(0).setDefaultOption("RedTest", "RedTest");
    autoChoosers.get(1).setDefaultOption("BlueTest", "BlueTest");

    SmartDashboard.putData("autoChooser", autoChoosers.get(autoChooserListID));

    trajectoryLoader.schedule();
  }

  private static void initTrajectory() throws FileNotFoundException {
    trajFile = Filesystem.getDeployDirectory().toPath().resolve("output/paths/" + autoFile).toFile();
    autoReader = new Scanner(trajFile);
    lines.clear();
    currentLine.clear();
    translation2ds.clear();
    rotation2ds.clear();
    pose2ds.clear();
    swerveControllerCommands.clear();
    fileOrder.clear();
    autoOrder.clear();
    SwerveDrive.gyro.reset();

    // Skip title line
    if (autoReader.hasNextLine()) {
      autoReader.nextLine();
    }
    // Parse entire auto file, and place each seperate line into an entry in the Lines list
    while (autoReader.hasNextLine()) {
      lines.add(autoReader.nextLine());
    }
    autoReader.close();
    // Pull all data entry-by-entry from the Lines list and add that data to new lists
    for (Integer i = 0; i <= lines.size() - 1; i++) {
      // Splits the next entry at each column, and adds that data to the CurrentLine list
      currentLine.addAll(Arrays.asList(lines.get(i).split(",")));
      // Skips adding the first "Move" string to the FileOrder list
      // This normalizes every series of "Move" entries to be 1 "Move" short of the actual number of points
      // This also helps because any commands on the first point will need to be done before the first move
      // So when we add the command here, it is before any "Move" entries
      if (i == 0) {
        if (currentLine.size() == 7) {
          fileOrder.add(currentLine.get(6));
          addPointToLists();
        }
      }
      else {
        fileOrder.add("Move");
        // If there is a command at the point, add the command to the FileOrder list
        // Also adds a copy of the current point to the lists
        // This simplifies later code by having points for both the endpoint of this move and the beginning of the next
        if (currentLine.size() == 7) {
          fileOrder.add(currentLine.get(6));
          addPointToLists();
        }
      }
      
      addPointToLists();
      currentLine.clear();
    }
    // Set the position of the odometry to the starting position of the auto
    SwerveDrive.odometry.resetPosition(SwerveDrive.gyroRotation2d,
        new SwerveModulePosition[] {SwerveDrive.frontRight.getPosition(),
            SwerveDrive.frontLeft.getPosition(), SwerveDrive.backLeft.getPosition(),
            SwerveDrive.backRight.getPosition()}, pose2ds.get(0));

    System.out.println(fileOrder);

    // Create all required SwerveControllerCommands, as well as a roadmap for what to do at each step of auto
    for (Integer i = 0; i <= fileOrder.size() - 1; i++) {
      // If the next command is to move, create a SwerveControllerCommand for every point up to the next non-move command
      if (fileOrder.get(i).equals("Move")) {
        // Store the starting index, since this is the beginning point of the move, then increment the index
        startIndex = i++;
        // Create the list of midpoints
        if (i <= fileOrder.size() - 2) {
          while (i <= fileOrder.size() - 2 & fileOrder.get(i).equals("Move")) {
            middlePoints.add(translation2ds.get(i++));
          }
          if (i <= fileOrder.size() - 1) {
            if (fileOrder.get(i).equals("Move")) {
              middlePoints.add(translation2ds.get(i++));
            }
          }
        }
        System.out.println(middlePoints.size());
        // Generate the trajectory, using the StartIndex for the starting position, the MiddlePoints list we just created, and the current index as the endpoint
        trajectory = TrajectoryGenerator.generateTrajectory(pose2ds.get(startIndex),
            middlePoints, pose2ds.get(i), trajConfig);
        // Generate the SwerveControllerCommand, and put it in the SwerveControllerCommandslist
        swerveControllerCommands.add(new SwerveControllerCommand(trajectory, SwerveDrive::getPose,
            SwerveDrive.kinematics, new HolonomicDriveController(new PIDController(3, 0, 0),
                new PIDController(3, 0, 0),
                new ProfiledPIDController(1.5, 0, 0, PIDConstraints)),
            SwerveDrive::setModuleStates));
        // Decrement the index in preparation for the for loop to increment it
        i--;
        middlePoints.clear();
      }
      // Add the command to the autoOrder list, which will act as a roadmap for auto
      autoOrder.add(fileOrder.get(i));
    }
    System.out.println(autoOrder);
  }

  private static void addPointToLists() {
    // Add the Translation2d of the point to the list
    translation2ds.add(new Translation2d(Double.parseDouble(currentLine.get(0)),
        Double.parseDouble(currentLine.get(1))));
    // Add the Rotation2d of the point to the list
    rotation2ds.add(new Rotation2d(Math.atan2(Double.parseDouble(currentLine.get(3)),
        Double.parseDouble(currentLine.get(2)))));
    System.out.println(Math.atan2(Double.parseDouble(currentLine.get(3)),
        Double.parseDouble(currentLine.get(2))));
    // Also add the Pose2d of the point to the list, for the endpoints
    pose2ds.add(new Pose2d(translation2ds.get(translation2ds.size() - 1),
        rotation2ds.get(rotation2ds.size() - 1)));
  }
}