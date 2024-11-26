package frc.robot.subsystems.drive;

import static frc.robot.subsystems.drive.DriveConstants.*;

import java.util.Optional;
import java.util.function.DoubleSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.DriveFeedforward;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.swerve.SwerveSetpoint;
import com.pathplanner.lib.util.swerve.SwerveSetpointGenerator;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.drive.controllers.HeadingController;
import frc.robot.subsystems.drive.controllers.ManualTeleopController;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.Vision.VisionObservation;
import frc.robot.utils.debugging.LoggedTunableNumber;
import frc.robot.utils.debugging.SysIDCharacterization;
import frc.robot.utils.math.AllianceFlipUtil;
import frc.robot.utils.swerve.LocalADStarAK;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Drive extends SubsystemBase {
    public static enum DriveState {
        TELEOP,
        AUTO_HEADING,
        SHOOT_ON_MOVE,
        AUTON,
        AUTO_ALIGN_AMP,
        SYSID_CHARACTERIZATION,
        WHEEL_CHARACTERIZATION,
        POV_UP,
        POV_DOWN,
        POV_RIGHT,
        POV_LEFT,
        STOP,
        AMP
    }

    private static final LoggedTunableNumber kDriftRate = 
        new LoggedTunableNumber("Drive/DriftRate", DriveConstants.kDriftRate);

    private Module[] modules;
    private GyroIO gyro;
    private GyroInputsAutoLogged gyroInputs = new GyroInputsAutoLogged();
    private Vision vision;
    private ChassisSpeeds desiredSpeeds = new ChassisSpeeds();
    private ChassisSpeeds ppDesiredSpeeds = new ChassisSpeeds();

    // private SwerveSetpoint currentSetpoint = new SwerveSetpoint(
    //     new ChassisSpeeds(), new SwerveModuleState[] {
    //         new SwerveModuleState(), new SwerveModuleState(),
    //         new SwerveModuleState(), new SwerveModuleState()
    //     });
    // private SwerveSetpointGenerator setpointGenerator;
    private RobotConfig robotConfig;
    private final SwerveSetpointGenerator setpointGenerator;
    private SwerveSetpoint previousSetpoint = new SwerveSetpoint(new ChassisSpeeds(0, 0, 0), 
        new SwerveModuleState[] {
            new SwerveModuleState(), new SwerveModuleState(),
            new SwerveModuleState(), new SwerveModuleState()
        }, new DriveFeedforward[0]);


    private Rotation2d robotRotation;
    private SwerveDriveOdometry odometry;
    private SwerveDrivePoseEstimator poseEstimator;
    private Field2d field = new Field2d();

    private PIDConstants translationPathplannerConstants = new PIDConstants(3.00, 0.0, 0.0);
    private PIDConstants rotationPathplannerConstants = new PIDConstants(1.0, 0.0, 0.0);
    private boolean PProtationTargetOverride = false;

    @AutoLogOutput(key="Drive/State")
    private DriveState driveState = DriveState.TELEOP;
    private boolean useGenerator = false;

    @AutoLogOutput(key="Drive/HeadingGoal")
    private Rotation2d headingGoal = new Rotation2d();

    private ManualTeleopController teleopController = new ManualTeleopController();
    private HeadingController headingController = new HeadingController();

    public Drive(Module[] modules, GyroIO gyro, Vision vision) {
        this.modules = modules;
        this.gyro = gyro;
        this.vision = vision;

        robotRotation = gyroInputs.yawPosition;

        odometry = new SwerveDriveOdometry
            (kKinematics, getRobotRotation(), getModulePositions());
        poseEstimator = new SwerveDrivePoseEstimator
            (kKinematics, getRobotRotation(), getModulePositions(), new Pose2d());

        try {
            robotConfig = RobotConfig.fromGUISettings();
        } catch(Exception e) {
            e.printStackTrace();
        }

        setpointGenerator = new SwerveSetpointGenerator(
            robotConfig, // The robot configuration. This is the same config used for generating trajectories and running path following commands.
            kMaxAzimuthAngularRadiansPS // The max rotation velocity of a swerve module in radians per second. This should probably be stored in your Constants file
        );

        AutoBuilder.configure(
            this::getPoseEstimate, 
            this::setPose, 
            this::getChassisSpeeds, 
            (speeds) -> ppDesiredSpeeds = speeds, 
            new PPHolonomicDriveController(
                translationPathplannerConstants, 
                rotationPathplannerConstants), 
            robotConfig, 
            () -> DriverStation.getAlliance().isPresent() && 
                DriverStation.getAlliance().get() == Alliance.Red, this);

        Pathfinding.setPathfinder(new LocalADStarAK());
        PathPlannerLogging.setLogActivePathCallback((activePath) -> Logger.recordOutput(
        "Drive/Odometry/Trajectory", activePath.toArray(new Pose2d[activePath.size()])));
        PathPlannerLogging.setLogTargetPoseCallback((targetPose) -> Logger.recordOutput(
        "Drive/Odometry/TrajectorySetpoint", targetPose));

        SmartDashboard.putData(field);

        headingController.setHeadingGoal(() -> headingGoal);
    }

    @Override
    public void periodic() {
        gyro.updateInputs(gyroInputs);
        Logger.processInputs("Drive/Gyro", gyroInputs);
        for (Module module : modules) {
            module.periodic();
            if (DriverStation.isDisabled()) {
                module.stop();
            }
        }

        if (gyroInputs.connected) {
            robotRotation = gyroInputs.yawPosition;
        } else {
            robotRotation = Rotation2d.fromRadians(
                (poseEstimator.getEstimatedPosition().getRotation().getRadians()
                    + getChassisSpeeds().omegaRadiansPerSecond * 0.02) % 360.0);
        }

        if(vision != null) {
            vision.periodic(poseEstimator.getEstimatedPosition(), odometry.getPoseMeters());
            VisionObservation[] observations = vision.getVisionObservations();
            for(VisionObservation observation : observations) {
                if(observation.hasObserved()) poseEstimator.addVisionMeasurement(
                    observation.pose(), 
                    observation.timeStamp(), 
                    observation.stdDevs());

                Logger.recordOutput(observation.camName()+"/stdDevX", observation.stdDevs().get(0));
                Logger.recordOutput(observation.camName()+"/stdDevY", observation.stdDevs().get(1));
                Logger.recordOutput(observation.camName()+"/stdDevTheta", observation.stdDevs().get(2));
                Logger.recordOutput(observation.camName()+"/Transform", odometry.getPoseMeters().minus(observation.pose()));
            }
        }

        poseEstimator.update(robotRotation, getModulePositions());
        odometry.update(robotRotation, getModulePositions());

        field.setRobotPose(getPoseEstimate());

        Logger.recordOutput("Drive/AmpPose", kAmpPose);
        ChassisSpeeds teleopSpeeds = teleopController
            .computeChassiSpeeds(poseEstimator.getEstimatedPosition().getRotation(), getChassisSpeeds());
        switch (driveState) {
                case TELEOP:
                    headingGoal = new Rotation2d();
                    desiredSpeeds = teleopSpeeds;
                    break;
            case POV_UP:
                desiredSpeeds = new ChassisSpeeds(0.5, 0.0, 
                    0.0);
                break;
            case POV_DOWN:
                desiredSpeeds = new ChassisSpeeds(-0.5, 0.0, 
                    0.0);
                break;
            case POV_RIGHT:
                desiredSpeeds = new ChassisSpeeds(0.0, 0.5, 
                    0.0);
                break;
            case POV_LEFT:
                desiredSpeeds = new ChassisSpeeds(0.0, -0.5, 
                    0.0);
                break;
            case AUTO_HEADING:
                headingGoal = getToSpeakerAngle();
                desiredSpeeds = new ChassisSpeeds(
                    teleopSpeeds.vxMetersPerSecond, teleopSpeeds.vyMetersPerSecond,
                    headingController.getSnapOutput(
                        poseEstimator.getEstimatedPosition().getRotation()));
                break;
            case AMP:
                headingGoal = Rotation2d.fromDegrees(-90.0);
                desiredSpeeds = new ChassisSpeeds(
                    teleopSpeeds.vxMetersPerSecond, teleopSpeeds.vyMetersPerSecond,
                    headingController.getSnapOutput(
                        poseEstimator.getEstimatedPosition().getRotation()));
            case SHOOT_ON_MOVE:
                break;
            case AUTON:
                if(RobotBase.isReal()) {
                    desiredSpeeds = new ChassisSpeeds(
                        ppDesiredSpeeds.vxMetersPerSecond, 
                        ppDesiredSpeeds.vyMetersPerSecond, 
                        -ppDesiredSpeeds.omegaRadiansPerSecond);
                } else {
                    desiredSpeeds = ppDesiredSpeeds;
                }
                break;
            case AUTO_ALIGN_AMP:
                desiredSpeeds = ppDesiredSpeeds;
                break;
            case SYSID_CHARACTERIZATION:
            case WHEEL_CHARACTERIZATION:
                desiredSpeeds = null;
                break;
            case STOP:
                // desiredSpeeds = new ChassisSpeeds();
                desiredSpeeds = null;
                for(Module module : modules) {
                    module.setDriveVoltage(0.0);
                }
                break;
            default:
                desiredSpeeds = null;
                break;
        }

        if (desiredSpeeds != null) {
            runSwerve(desiredSpeeds);
        }

        SmartDashboard.putData(field);
    }

    public Command followFirstChoreoPath(String pathName, Rotation2d startingRotation) {
        PathPlannerPath path = getTraj(pathName).get();
        double totalTimeSeconds = path.getIdealTrajectory(robotConfig).get().getTotalTimeSeconds();
        return 
            new SequentialCommandGroup(
                new InstantCommand(() -> {
                    setDriveState(DriveState.AUTON);
                    setPose(new Pose2d(
                        AllianceFlipUtil.apply(path.getPathPoses().get(0).getX()), 
                        path.getPathPoses().get(0).getY(), 
                        AllianceFlipUtil.apply(path.getPathPoses().get(0).getRotation())
                    ));
                }), 
                AutoBuilder.followPath(path).withTimeout(totalTimeSeconds + 0.5), 
                setDriveStateCommand(DriveState.STOP));
    }

    public Command followChoreoPath(String pathName) {
        PathPlannerPath path = getTraj(pathName).get();
        path.getIdealTrajectory(null);
        double totalTimeSeconds = path.getIdealTrajectory(robotConfig).get().getTotalTimeSeconds();
        return 
            setDriveStateCommand(DriveState.AUTON).andThen(
                AutoBuilder.followPath(path).withTimeout(totalTimeSeconds + 0.5), 
                setDriveStateCommand(DriveState.STOP));
    }

    public Optional<PathPlannerPath> getTraj(String pathName) {
        try {
            return Optional.of(PathPlannerPath.fromChoreoTrajectory(pathName));
        } catch(Exception e) {
            e.printStackTrace();
            return Optional.empty();
        }
    }

    public boolean inBetween(double val, double range) {
        return Math.abs(val) < range;
    }

    ///////////////////////// STATE SETTING \\\\\\\\\\\\\\\\\\\\\\\\
    public Command setDriveStateCommand(DriveState state) {
        return Commands.runOnce(() -> setDriveState(state), this);
    }

    public Command setDriveStateCommandContinued(DriveState state) {
        return new FunctionalCommand(
            () -> setDriveState(state), 
            () -> {}, 
            (interrupted) -> {}, 
            () -> false, 
            this);
    }

    public void setDriveState(DriveState state) {
        driveState = state;
        if(driveState != null) {
            if(state.equals(DriveState.AUTO_HEADING)) {
                headingController.resetController(
                    robotRotation, Rotation2d.fromRadians(gyroInputs.yawVelocityRadiansPerSecond));
                headingGoal = getToSpeakerAngle();
            } else if(
                state.equals(DriveState.POV_LEFT) || state.equals(DriveState.POV_RIGHT) || 
                state.equals(DriveState.POV_UP) || state.equals(DriveState.POV_DOWN)) {
                    headingGoal = robotRotation;
            };
        }
    }

    public DriveState getDriveState() {
        return driveState;
    }

    public void setChassisSpeeds(ChassisSpeeds speeds) {
        desiredSpeeds = speeds;
    }

    public Command pathFindToAmp() {
        return Commands.runOnce( () -> new SequentialCommandGroup(
            setDriveStateCommand(DriveState.AUTO_ALIGN_AMP),
            AutoBuilder.pathfindToPose( new Pose2d(
                AllianceFlipUtil.apply(kAmpPose.getX()), 
                kAmpPose.getY() - (kDrivebaseRadiusMeters / 2.0 + 1.0), 
                kAmpPose.getRotation()),
                kAutoAlignConstraints,
                2.0),
            AutoBuilder.pathfindToPose( new Pose2d(
                AllianceFlipUtil.apply(kAmpPose.getX()),
                kAmpPose.getY() - kDrivebaseRadiusMeters / 2.0,
                kAmpPose.getRotation()),
                kAutoAlignConstraints ) ).schedule() );
    }

    public Command characterizeDriveMotors() {
        return setDriveStateCommand(DriveState.SYSID_CHARACTERIZATION).andThen(
            SysIDCharacterization.runDriveSysIDTests( (voltage) -> {
                for (var module : modules) module.runLinearCharacterization(voltage);
        }, this));
    }

    public Command characterizeAngular() {
        return setDriveStateCommand(DriveState.SYSID_CHARACTERIZATION).andThen(
            SysIDCharacterization.runDriveSysIDTests( (voltage) -> runMOICharacterization(voltage), this));
    }

    public Command characterizeDriveBaseMOI() {
        return SysIDCharacterization.MOISYSid(
            (v) -> runMOICharacterization(v), 
            () -> new double[] {
                modules[0].getInputs().azimuthStatorCurrentAmps[0],
                modules[1].getInputs().azimuthStatorCurrentAmps[0],
                modules[2].getInputs().azimuthStatorCurrentAmps[0],
                modules[3].getInputs().azimuthStatorCurrentAmps[0]
            }, this);
    }

    ////////////// CHASSIS SPEEDS \\\\\\\\\\\\\\\\
    public void runSwerve(ChassisSpeeds speeds) {
        desiredSpeeds = discretize(speeds);
        // FOR LOGGING
        SwerveModuleState[] unoptimizedSetpointStates = kKinematics.toSwerveModuleStates(desiredSpeeds);

        SwerveModuleState[] setpointStates = kKinematics.toSwerveModuleStates(desiredSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(unoptimizedSetpointStates, kMaxLinearSpeedMPS);
        SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, kMaxLinearSpeedMPS);

        SwerveModuleState[] optimizedSetpointStates = new SwerveModuleState[4];

        previousSetpoint = setpointGenerator.generateSetpoint(
            previousSetpoint, desiredSpeeds, 0.02);

        for (int i = 0; i < 4; i++) {
            if(useGenerator) {
                setpointStates[i] = new SwerveModuleState(
                    previousSetpoint.moduleStates()[i].speedMetersPerSecond,
                    Math.abs(previousSetpoint.moduleStates()[i].speedMetersPerSecond / kMaxLinearSpeedMPS) < 0.01 ?
                    modules[i].getCurrentState().angle : previousSetpoint.moduleStates()[i].angle);

                Logger.recordOutput("Drive/Swerve/Feedforward/"+i+"/Acceleration", previousSetpoint.feedforwards()[i].accelerationMPS());
                Logger.recordOutput("Drive/Swerve/Feedforward/"+i+"/Force", previousSetpoint.feedforwards()[i].forceNewtons());
                Logger.recordOutput("Drive/Swerve/Feedforward/"+i+"/Current", previousSetpoint.feedforwards()[i].torqueCurrentAmps());

                if(kUsingKrakens) {
                    optimizedSetpointStates[i] = modules[i].setDesiredStateWithAccel(SwerveModuleState.optimize(
                        setpointStates[i], modules[i].getCurrentState().angle), previousSetpoint.feedforwards()[i].torqueCurrentAmps());
                } else {
                    optimizedSetpointStates[i] = modules[i].setDesiredStateWithAccel(SwerveModuleState.optimize(
                        setpointStates[i], modules[i].getCurrentState().angle), previousSetpoint.feedforwards()[i].accelerationMPS());
                }
            } else {
                setpointStates[i] = new SwerveModuleState(
                    previousSetpoint.moduleStates()[i].speedMetersPerSecond,
                    Math.abs(previousSetpoint.moduleStates()[i].speedMetersPerSecond / kMaxLinearSpeedMPS) < 0.01 ?
                    modules[i].getCurrentState().angle : previousSetpoint.moduleStates()[i].angle);

                optimizedSetpointStates[i] = modules[i].setDesiredState(SwerveModuleState.optimize(
                    unoptimizedSetpointStates[i], modules[i].getCurrentState().angle));
            }
        }
        
        Logger.recordOutput("Drive/Swerve/Setpoints", unoptimizedSetpointStates);
        Logger.recordOutput("Drive/Swerve/SetpointsOptimized", optimizedSetpointStates);
        Logger.recordOutput("Drive/Swerve/SetpointsChassisSpeeds", kKinematics.toChassisSpeeds(optimizedSetpointStates));
    }

    public void setAmperagesForAllModules(double amps) {
        for(int i = 0; i < 4; i++) {
            modules[i].setDesiredVelocity(null);
            modules[i].setDriveAmp(amps);
            modules[i].setDesiredRotation(Rotation2d.fromDegrees(0.0));
        }
    }

    private ChassisSpeeds discretize(ChassisSpeeds speeds) {
        double dt = 0.02;
        var desiredDeltaPose = new Pose2d(
            speeds.vxMetersPerSecond * dt,
            speeds.vyMetersPerSecond * dt,
            new Rotation2d(speeds.omegaRadiansPerSecond * dt * kDriftRate.get()));
        var twist = new Pose2d().log(desiredDeltaPose);

        return new ChassisSpeeds((twist.dx / dt), (twist.dy / dt), (speeds.omegaRadiansPerSecond));
    }

    public void stop() {
        runSwerve(new ChassisSpeeds());
    }

    ////////////// LOCALIZATION \\\\\\\\\\\\\\\\
    public void resetGyro() {
        robotRotation = Constants.kAlliance == Alliance.Blue ? 
            Rotation2d.fromDegrees(0.0) : Rotation2d.fromDegrees(180.0);
        gyro.resetGyro(robotRotation);
        setPose(new Pose2d(getPoseEstimate().getTranslation(), robotRotation));
    }

    public void resetPose() {
        setPose(new Pose2d());
    }

    public void setPose(Pose2d pose) {
        robotRotation = pose.getRotation();
        gyro.resetGyro(robotRotation);
        poseEstimator.resetPosition(getRobotRotation(), getModulePositions(), pose);
        odometry.resetPosition(getRobotRotation(), getModulePositions(), pose);
    }

    public void addVisionMeasurement(
        Pose2d visionMeasurement, double timestampS, Matrix<N3, N1> stdDevs) {
        poseEstimator.addVisionMeasurement(visionMeasurement, timestampS, stdDevs);
    }

    // ONLY FOR TESTING
    public void setPoses(Pose2d visionPose, Pose2d odometryPose) {
        robotRotation = visionPose.getRotation();
        gyro.resetGyro(robotRotation);
        // Safe to pass in odometry poses because of the syncing
        // between gyro and pose estimator in reset gyro function
        poseEstimator.resetPosition(getRobotRotation(), getModulePositions(), visionPose);
        odometry.resetPosition(getRobotRotation(), getModulePositions(), odometryPose);
    }

    ////////////// GETTERS \\\\\\\\\\\\\\\\
    @AutoLogOutput(key = "Drive/Swerve/ToSpeakerAngle")
    public Rotation2d getToSpeakerAngle() {
        Pose2d adjustedSpeakerPose = new Pose2d(
            AllianceFlipUtil.apply(kSpeakerPose.getX()), 
            kSpeakerPose.getY(), Rotation2d.fromDegrees(0.0));
        return new Rotation2d(
            getPoseEstimate().getX() - adjustedSpeakerPose.getX(), 
            getPoseEstimate().getY() - adjustedSpeakerPose.getY())
                .plus(Rotation2d.fromDegrees(0.0));
    }

    @AutoLogOutput(key = "Drive/Swerve/ToSpeakerDistance")
    public double getDistanceToSpeakerMeters() {
        Pose2d adjustedSpeakerPose = new Pose2d(
            AllianceFlipUtil.apply(kSpeakerPose.getX()), 
            kSpeakerPose.getY(), Rotation2d.fromDegrees(0.0));
        return Math.hypot(
            getPoseEstimate().getX() - adjustedSpeakerPose.getX(),
            getPoseEstimate().getY() - adjustedSpeakerPose.getY());
    }

    @AutoLogOutput(key = "Drive/Swerve/MeasuredStates")
    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (int i = 0; i < 4; i++) states[i] = modules[i].getCurrentState();

        return states;
    }

    @AutoLogOutput(key = "Drive/Swerve/ModulePositions")
    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (int i = 0; i < 4; i++) positions[i] = modules[i].getCurrentPosition();

        return positions;
    }

    // TODO: UNDO ISREAL CONDITION LATER
    @AutoLogOutput(key = "Drive/Odometry/PoseEstimate")
    public Pose2d getPoseEstimate() {
        return RobotBase.isReal() ? poseEstimator.getEstimatedPosition() : odometry.getPoseMeters();
    }

    @AutoLogOutput(key = "Drive/Odometry/DrivePose")
    public Pose2d getOdometryPose() {
        return odometry.getPoseMeters();
    }

    @AutoLogOutput(key = "Drive/Odometry/RobotRotation")
    public Rotation2d getRobotRotation() {
        return robotRotation;
    }

    @AutoLogOutput(key = "Drive/Odometry/GyroRotation")
    public Rotation2d getGyroRotation() {
        return gyroInputs.yawPosition;
    }

    @AutoLogOutput(key = "Drive/Odometry/CurrentChassisSpeeds")
    public ChassisSpeeds getChassisSpeeds() {
        return kKinematics.toChassisSpeeds(getModuleStates());
    }

    @AutoLogOutput(key = "Drive/Odometry/DesiredChassisSpeeds")
    public ChassisSpeeds getDesiredChassisSpeeds() {
        return desiredSpeeds;
    }

    @AutoLogOutput(key = "Drive/PP/RotationTargetOverride")
    public boolean getPPRotationTargetOverride() {
        return PProtationTargetOverride;
    }

    @AutoLogOutput
    public boolean inHeadingTolerance() {
        return Math.min(
            Math.min(
                Math.abs(headingGoal.minus(getRobotRotation()).getDegrees()),
                Math.abs(headingGoal.minus(Rotation2d.fromDegrees(360.0)).minus(getRobotRotation()).getDegrees())),
            Math.abs(headingGoal.minus(getRobotRotation().minus(Rotation2d.fromDegrees(360.0))).getDegrees())
            ) < 1.0;
    }

    ////////////// SETTERS \\\\\\\\\\\\\\\\
    public void setPProtationTargetOverride(boolean override) {
        PProtationTargetOverride = override;
    }

    public void resetModulesEncoders() {
        for (int i = 0; i < 4; i++) modules[i].resetAzimuthEncoder();
    }

    public void runCharacterizationForAllModules(double volts) {
        setDriveState(DriveState.SYSID_CHARACTERIZATION);
        for(int i = 0; i < 4; i++) modules[i].runLinearCharacterization(volts);
    }

    public void runMOICharacterization(double volts) {
        setDriveState(DriveState.SYSID_CHARACTERIZATION);
        for(int i = 0; i < 4; i++) {
            switch (i) {
                case 0:
                    modules[0].runCircularCharacterization(volts, Rotation2d.fromDegrees(-45.0));
                    break;
                case 1:
                    modules[1].runCircularCharacterization(-volts, Rotation2d.fromDegrees(45.0));
                case 2:
                    modules[2].runCircularCharacterization(volts, Rotation2d.fromDegrees(45.0));
                case 3:
                    modules[3].runCircularCharacterization(-volts, Rotation2d.fromDegrees(-45.0));
                default:
                    break;
            }
        }
    }

    public void acceptJoystickInputs(DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier thetaSupplier) {
        teleopController.acceptJoystickInputs(xSupplier, ySupplier, thetaSupplier);
    }

    public void setVision(Vision robotVision) {
        vision = robotVision;
    }
}
