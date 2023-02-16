package frc.robot.subsystems;

import java.util.ArrayList;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
// import com.ctre.phoenix.sensors.Pigeon2;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.wpiClasses.QuadSwerveSim;


public class Swerve extends SubsystemBase {
  private final AHRS gyro = new AHRS(Port.kMXP, (byte) 200); // NavX connected over MXP port

  private SwerveDriveOdometry swerveOdometry;
  private SwerveModule m_frontLeftModule, m_frontRightModule, m_rearLeftModule, m_rearRightModule;
  private SwerveModule[] mSwerveMods;


  public Field2d field;

  public Swerve() {
    zeroGyro();

    m_frontLeftModule = new SwerveModule(0, Constants.Swerve.Mod0.constants);
    m_frontRightModule = new SwerveModule(1, Constants.Swerve.Mod1.constants);
    m_rearLeftModule = new SwerveModule(2, Constants.Swerve.Mod2.constants);
    m_rearRightModule = new SwerveModule(3, Constants.Swerve.Mod3.constants);

    mSwerveMods = new SwerveModule[] {
        m_frontLeftModule,
        m_frontRightModule,
        m_rearLeftModule,
        m_rearRightModule
    };

    swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getYaw(), getPositions());
    field = new Field2d();
    SmartDashboard.putData("Field", field);
  }

  public Field2d getField() {
    return field;
  }

  // public void setField(Field2d field) {
  // this.field = field;
  // }

  public void drive(
      Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
    SwerveModuleState[] swerveModuleStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(
                translation.getX(), translation.getY(), rotation, getYaw())
            : new ChassisSpeeds(translation.getX(), translation.getY(), rotation));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

    for (SwerveModule mod : mSwerveMods) {
      mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
    }
  }

  /* Used by SwerveControllerCommand in Auto */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);
    for (SwerveModule mod : mSwerveMods) {
      mod.setDesiredState(desiredStates[mod.moduleNumber], false);
    }
  }

  public Pose2d getPose() {
    return swerveOdometry.getPoseMeters();
  }

  public Translation2d getTranslation() {
    Pose2d currentPose2d = new Pose2d();
    Translation2d currentTranslaton2d = new Translation2d();
    currentPose2d = swerveOdometry.getPoseMeters();
    currentTranslaton2d = currentPose2d.getTranslation();
    return currentTranslaton2d;
  };

  public void resetOdometry(Pose2d pose) {
    swerveOdometry.resetPosition(getGyroscopeRotation(), getPositions(), pose);
  }

  public SwerveModuleState[] getStates() {
    SwerveModuleState[] states = new SwerveModuleState[]{
      m_frontLeftModule.getState(),
      m_frontRightModule.getState(),
      m_rearLeftModule.getState(),
      m_rearRightModule.getState()
    };
    return states;
  };

  public SwerveModulePosition[] getPositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[] {
      m_frontLeftModule.getPosition(),
      m_frontRightModule.getPosition(),
      m_rearLeftModule.getPosition(),
      m_rearRightModule.getPosition()
    };
    return positions;
  };



  public void stopModules() {
    m_frontLeftModule.setDesiredState(new SwerveModuleState(0, new Rotation2d(0)), false);
    m_frontRightModule.setDesiredState(new SwerveModuleState(0, new Rotation2d(0)), false);
    m_rearLeftModule.setDesiredState(new SwerveModuleState(0, new Rotation2d(0)), false);
    m_rearRightModule.setDesiredState(new SwerveModuleState(0, new Rotation2d(0)), false);

  }

  public void zeroGyro() {
    gyro.zeroYaw();
  }

  public Rotation2d getYaw() {
    return (Constants.Swerve.invertGyro)
        ? Rotation2d.fromDegrees(360 - heading())
        : Rotation2d.fromDegrees(heading());
  }

  public Rotation2d getGyroscopeRotation() {
    if (gyro.isMagnetometerCalibrated()) {
      // We will only get valid fused headings if the magnetometer is calibrated
      return Rotation2d.fromDegrees(gyro.getFusedHeading());
    }
    // // We have to invert the angle of the NavX so that rotating the robot
    // counter-clockwise makes the angle increase.
    return Rotation2d.fromDegrees(360.0 - gyro.getYaw());
  }

  public double heading() {
    if (gyro.isMagnetometerCalibrated()) {
      // We will only get valid fused headings if the magnetometer is calibrated
      return (gyro.getFusedHeading());
    }
    // // We have to invert the angle of the NavX so that rotating the robot
    // counter-clockwise makes the angle increase.
    return (360.0 - gyro.getYaw());

  }

  
  @Override
  public void periodic() {
    swerveOdometry.update(getGyroscopeRotation(), getPositions());
    field.setRobotPose(getPose());
    SmartDashboard.putNumber("Robot Heading", heading());
    SmartDashboard.putString("Robot Location", getPose().getTranslation().toString());

    for (SwerveModule mod : mSwerveMods) {
      SmartDashboard.putNumber(
          "Mod " + mod.moduleNumber + " Cancoder", mod.getCanCoder().getDegrees());
      SmartDashboard.putNumber(
          "Mod " + mod.moduleNumber + " Integrated", mod.getState().angle.getDegrees());
      SmartDashboard.putNumber(
          "Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);
    }
    SmartDashboard.updateValues();

  }
}