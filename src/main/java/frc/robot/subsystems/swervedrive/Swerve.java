package frc.robot.subsystems.swervedrive;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.subsystems.swervedrive.SwerveConstants.*;

/**
 * Represents the Swerve drive subsystem, managing the kinematics, odometry, and
 * control of swerve modules, gyro, and pose estimation for an FRC robot.
 */
public class Swerve extends SubsystemBase {
    // Swerve drive odometry, managing robot's position based on movement data
    public SwerveDriveOdometry swerveOdometry;
    // Array of SwerveModule objects, one for each wheel/module on the robot
    public SwerveModule[] mSwerveMods;
    // Gyro sensor for tracking robot orientation
    public Pigeon2 gyro;
    // Pose estimator for managing robot's estimated position on the field
    public SwerveDrivePoseEstimator poseEstimator;
    
    /**
     * Constructs a new Swerve subsystem, initializing swerve modules, gyro, odometry,
     * pose estimator, and AutoBuilder with configurations for autonomous path following.
     */
    public Swerve() {
        // Initialize gyro with ID and device name
        gyro = new Pigeon2(kPigeonID, "drivebase");
        gyro.getConfigurator();  // Retrieve gyro configuration settings
        zeroGyro();  // Set initial yaw angle to 0
        
        // Initialize swerve modules with unique identifiers and constants
        mSwerveMods = new SwerveModule[] {
            new SwerveModule(0, Mod1),
            new SwerveModule(1, Mod2),
            new SwerveModule(2, Mod3),
            new SwerveModule(3, Mod4)
        };
        
        // Set up odometry and pose estimator with initial yaw and module positions
        swerveOdometry = new SwerveDriveOdometry(kSwerveKinematics, getYaw(), getModulePositions());
        poseEstimator = new SwerveDrivePoseEstimator(kSwerveKinematics, getYaw(), getModulePositions(), getPose());
        
        // Configure autonomous path following for holonomic swerve
        AutoBuilder.configureHolonomic(
            this::getPose, // Supplier for current robot pose
            this::resetOdometry, // Method to reset odometry with starting pose in auto
            this::getChassisSpeeds, // Supplier for chassis speeds relative to the robot
            this::setModuleSpeeds, // Method to set module speeds for the robot
            new HolonomicPathFollowerConfig( // Config for path following
                    new PIDConstants(kDriveP, 0.0, 0.0), // PID for drive translation
                    new PIDConstants(kAzimuthP, 0.0, 0.0), // PID for rotation control
                    kMaxSpeed, // Max speed for swerve modules
                    kWheelBase, // Radius of the drivebase (center to module distance)
                    new ReplanningConfig(true, true) // Path replanning settings
            ),
            () -> {
              // Determines if the path will be mirrored for the red alliance
              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            this // Reference to this subsystem for setting command requirements
        );
    }
    
    /**
     * Drives the robot using translation, rotation, and an open-loop option.
     * Calculates the desired states for each swerve module.
     */
    public void drive(Translation2d translation, double rotation, boolean isOpenLoop) {
        // Convert field-relative speeds to module-relative speeds
        SwerveModuleState[] swerveModuleStates = 
        kSwerveKinematics.toSwerveModuleStates(
            ChassisSpeeds.fromFieldRelativeSpeeds(
                translation.getX(), 
                translation.getY(), 
                rotation, 
                getYaw()));

        // Adjust wheel speeds to avoid exceeding max speed
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, kMaxSpeed);
        
        // Set desired states for each swerve module based on open/closed-loop control
        for(SwerveModule mod : mSwerveMods) {
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    } 
    
    /**
     * Directly sets the states for each swerve module with closed-loop control.
     */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        // Desaturate module speeds to avoid exceeding max
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, kMaxSpeed);
        
        // Apply desired state to each module
        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }    

    /**
     * Sets the speeds for each swerve module based on chassis speeds.
     */
    public void setModuleSpeeds(ChassisSpeeds desiredSpeeds){
        SwerveDriveKinematics.desaturateWheelSpeeds(kSwerveKinematics.toSwerveModuleStates(desiredSpeeds), kMaxSpeed);

        int i = 0;
        // Set module speeds based on chassis speeds for each module
        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(kSwerveKinematics.toSwerveModuleStates(desiredSpeeds)[i], false);
            i += 1;
        }
    }

    /**
     * Resets the gyro yaw to 0.
     */
    public void resetGyro(){
        gyro.setYaw(0.0);
    }

    /**
     * Returns the robot's current pose (position and orientation).
     */
    public Pose2d getPose() {
        return swerveOdometry.getPoseMeters();
    }

    /**
     * Resets the odometry to a specific pose.
     */
    public void resetOdometry(Pose2d pose) {
        swerveOdometry.resetPosition(getYaw(), getModulePositions(), pose);
    }

    /**
     * Retrieves the current states for each swerve module.
     */
    public SwerveModuleState[] getStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveModule mod : mSwerveMods) {
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    /**
     * Retrieves the robot's current chassis speeds.
     */
    public ChassisSpeeds getChassisSpeeds() {
        SwerveModuleState[] states = getStates();
        return kSwerveKinematics.toChassisSpeeds(
            states[0], states[1], states[2], states[3]
        );
    }
    
    /**
     * Returns the current position of each swerve module.
     */
    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (SwerveModule mod : mSwerveMods){
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    /**
     * Sets the gyro yaw angle to zero, aligning it with the field.
     */
    public void zeroGyro() {
        gyro.setYaw(0);
    }
    
    /**
     * Manually sets the gyro yaw to a specific angle.
     */
    public void setGyro(double degrees) {
        gyro.setYaw(degrees);
    }

    /**
     * Retrieves the current yaw of the robot, taking into account any inversion settings.
     */
    public Rotation2d getYaw() {
        return (kInvertGyro) ? Rotation2d.fromDegrees(360 - gyro.getYaw().getValueAsDouble()) : Rotation2d.fromDegrees(gyro.getYaw().getValueAsDouble());
    }

    /**
     * Resets each module's motor position to its absolute encoder value.
     */
    public void resetAllMotors() {
        for(int i = 0; i < 4; i++) mSwerveMods[i].resetToAbsolute();
    }

    /**
     * Periodically updates the SmartDashboard with the current yaw angle.
     */
    @Override
    public void periodic() {
        SmartDashboard.putNumber("Drive / Yaw", getYaw().getDegrees());
    }
}
