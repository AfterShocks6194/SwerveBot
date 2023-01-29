// package frc.robot.commands;

// import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.controller.ProfiledPIDController;
// import edu.wpi.first.math.filter.SlewRateLimiter;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.wpilibj2.command.CommandBase;
// import edu.wpi.first.wpilibj2.command.InstantCommand;
// import edu.wpi.first.wpilibj2.command.RunCommand;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
// import frc.robot.Constants;
// import frc.robot.subsystems.Swerve;

// import com.pathplanner.lib.PathConstraints;
// import com.pathplanner.lib.PathPlanner;
// import com.pathplanner.lib.PathPlannerTrajectory;
// import com.pathplanner.lib.PathPoint;
// import com.pathplanner.lib.commands.PPSwerveControllerCommand;

// public class GoScore extends CommandBase {
//   private Swerve s_Swerve;


//   public GoScore(
//       Swerve s_Swerve) {
//     this.s_Swerve = s_Swerve;
//     addRequirements(s_Swerve);
//   }

//   @Override
//   public void initialize() {
//    PathPlannerTrajectory onTheFly = new PathPlannerTrajectory();
//     onTheFly = PathPlanner.generatePath(
//       new PathConstraints(4, 3),
//       new PathPoint(s_Swerve.getTranslation(), s_Swerve.getGyroscopeRotation()),
//       new PathPoint(new Translation2d(2.82, 4.67), Rotation2d.fromDegrees(180), Rotation2d.fromDegrees(180)));

//   PIDController xController = new PIDController(Constants.AutoConstants.kPXController, 0, 0);
//   PIDController yController = new PIDController(Constants.AutoConstants.kPYController, 0, 0);
//   ProfiledPIDController thetaController = new ProfiledPIDController(
//       Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
//   thetaController.enableContinuousInput(-Math.PI, Math.PI);

//   }

//   @Override
//   public void execute() {
//     PPSwerveControllerCommand swerveControllerCommand = new PPSwerveControllerCommand(
//       onTheFly,
//       s_Swerve::getPose,
//       Constants.Swerve.swerveKinematics,
//       xController,
//       yController,
//       yController,
//       s_Swerve::setModuleStates,
//       true,
//       s_Swerve);

//       s_Swerve.resetOdometry(onTheFly.getInitialHolonomicPose());
//   }


//   @Override
//   public boolean isFinished() {
//     return true;
//   }

  


// }
