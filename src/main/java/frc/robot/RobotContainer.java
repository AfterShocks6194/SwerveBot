// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.photonvision.PhotonCamera;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.autos.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.Constants;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  /* Controllers */
  private final Joystick driver = new Joystick(0);

  /* Drive Controls */
  private final int translationAxis = XboxController.Axis.kLeftY.value;
  private final int strafeAxis = XboxController.Axis.kLeftX.value;
  private final int rotationAxis = XboxController.Axis.kRightX.value;

  /* Driver Buttons */
  private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);
  private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
  private final JoystickButton getGamepiece = new JoystickButton(driver, XboxController.Button.kA.value);
  private final JoystickButton goScore = new JoystickButton(driver, XboxController.Button.kB.value);


  /* Subsystems */
  private final Swerve s_Swerve = new Swerve();

  TrajectoryConfig defaultConfig;

  private static final SendableChooser<String> AutoPath = new SendableChooser<>();

  private final PhotonCamera photonCamera = new PhotonCamera("LimeLight");
  private final PoseEstimatorSubsystem poseEstimator = new PoseEstimatorSubsystem(photonCamera, s_Swerve);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    s_Swerve.setDefaultCommand(
        new TeleopSwerve(
            s_Swerve,
            () -> driver.getRawAxis(translationAxis),
            () -> driver.getRawAxis(strafeAxis),
            () -> driver.getRawAxis(rotationAxis),
            () -> robotCentric.getAsBoolean()));

    // Configure the button bindings
    configureButtonBindings();

    AutoPath.addOption("New Path", "New Path");
    AutoPath.addOption("Testing", "Testing");
    AutoPath.setDefaultOption("New Path", "New Path");
    SmartDashboard.putData("Auto Pathing", AutoPath);

  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
   * it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    /* Driver Buttons */
    zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));
    getGamepiece.onTrue(goGetGamePieces());
    goScore.onTrue(returnToScore());

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    // return new exampleAuto(s_Swerve);

    PathPlannerTrajectory ChosenPath = PathPlanner.loadPath(AutoPath.getSelected(), 2, 2);
    s_Swerve.field.getObject("traj").setTrajectory(ChosenPath);
    s_Swerve.field.setRobotPose(ChosenPath.getInitialHolonomicPose());

    // 3. Define PID controllers for tracking trajectory
    PIDController xController = new PIDController(Constants.AutoConstants.kPXController, 0, 0);
    PIDController yController = new PIDController(Constants.AutoConstants.kPYController, 0, 0);
    ProfiledPIDController thetaController = new ProfiledPIDController(
        Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    PPSwerveControllerCommand swerveControllerCommand = new PPSwerveControllerCommand(
        ChosenPath,
        poseEstimator::getCurrentPose,
        Constants.Swerve.swerveKinematics,
        xController,
        yController,
        yController,
        s_Swerve::setModuleStates,
        true,
        s_Swerve);

    // 5. Add some init and wrap-up, and return everything
    return new SequentialCommandGroup(
        new InstantCommand(() -> s_Swerve.resetOdometry(ChosenPath.getInitialHolonomicPose())),
        swerveControllerCommand,
        new InstantCommand(() -> s_Swerve.stopModules()));

  }

  public Command goGetGamePieces() {
    PathPlannerTrajectory onTheFly = PathPlanner.generatePath(
        new PathConstraints(4, 3),
        new PathPoint(s_Swerve.getTranslation(), s_Swerve.getGyroscopeRotation()),
        new PathPoint(new Translation2d(14.75, 7.35), Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0)));

    PIDController xController = new PIDController(Constants.AutoConstants.kPXController, 0, 0);
    PIDController yController = new PIDController(Constants.AutoConstants.kPYController, 0, 0);
    ProfiledPIDController thetaController = new ProfiledPIDController(
        Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    PPSwerveControllerCommand swerveControllerCommand = new PPSwerveControllerCommand(
        onTheFly,
        poseEstimator::getCurrentPose,
        Constants.Swerve.swerveKinematics,
        xController,
        yController,
        yController,
        s_Swerve::setModuleStates,
        true,
        s_Swerve);

    return new SequentialCommandGroup(
        new InstantCommand(() -> s_Swerve.resetOdometry(onTheFly.getInitialHolonomicPose())),
        swerveControllerCommand,
        new InstantCommand(() -> s_Swerve.stopModules()));
  }


  public Command returnToScore() {
    PathPlannerTrajectory onTheFly = PathPlanner.generatePath(
        new PathConstraints(4, 3),
        new PathPoint(s_Swerve.getTranslation(), s_Swerve.getGyroscopeRotation()),
        new PathPoint(new Translation2d(2.82, 4.67), Rotation2d.fromDegrees(180), Rotation2d.fromDegrees(180)));

    PIDController xController = new PIDController(Constants.AutoConstants.kPXController, 0, 0);
    PIDController yController = new PIDController(Constants.AutoConstants.kPYController, 0, 0);
    ProfiledPIDController thetaController = new ProfiledPIDController(
        Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    PPSwerveControllerCommand swerveControllerCommand = new PPSwerveControllerCommand(
        onTheFly,
        poseEstimator::getCurrentPose,
        Constants.Swerve.swerveKinematics,
        xController,
        yController,
        yController,
        s_Swerve::setModuleStates,
        true,
        s_Swerve);

    return new SequentialCommandGroup(
        new InstantCommand(() -> s_Swerve.resetOdometry(onTheFly.getInitialHolonomicPose())),
        swerveControllerCommand,
        new InstantCommand(() -> s_Swerve.stopModules()));

  }

  



}
