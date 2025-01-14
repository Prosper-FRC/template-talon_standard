package frc.robot.swerve;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.*;
import com.pathplanner.lib.util.swerve.SwerveSetpoint;
import com.pathplanner.lib.util.swerve.SwerveSetpointGenerator;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import frc.robot.swerve.SwerveModule.SwerveModule;
import frc.robot.swerve.controllers.HeadingController;
import frc.robot.swerve.controllers.TeleopController;
import frc.robot.swerve.gyro.GyroHardware;
import frc.robot.swerve.gyro.GyroInputsAutoLogged;
import frc.robot.utils.debugging.LoggedTunableNumber;
import frc.robot.utils.debugging.SysIDCharacterization;
import frc.robot.utils.swerve.LocalADStarAK;

public class Drive extends SubsystemBase{

    public static enum DriveState {
        TELEOP,
        AUTO_HEADING,
        AUTON,
        SYS_ID,
        SNIPER_UP,
        SNIPER_DOWN,
        SNIPER_RIGHT,
        SNIPER_LEFT,
        STOP
    }

    private SwerveModule[] modules; 
    private GyroHardware gyro;
    private GyroInputsAutoLogged gyroInputs = new GyroInputsAutoLogged();
    
    private ChassisSpeeds desiredSpeeds = new ChassisSpeeds();
    private ChassisSpeeds autonDesiredSpeeds = new ChassisSpeeds();

    private Rotation2d robotRotation;
    private SwerveDriveOdometry swerveOdometry;
    private SwerveDrivePoseEstimator swervePoseEstimator;
    private Field2d field = new Field2d();

    private TeleopController teleopController = new TeleopController();
    private HeadingController headingController = new HeadingController();

    private RobotConfig robotConfig;
    private SwerveSetpointGenerator generator;
    private SwerveSetpoint previousSetpoint;
    private PIDConstants translationPathplannerConstants = new PIDConstants(0.0, 0.0, 0.0);
    private PIDConstants rotationPathplannerConstants = new PIDConstants(0.0, 0.0, 0.0);
    private boolean useGenerator = false;

    private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(DriveConstants.kModuleTranslations);

    private static final LoggedTunableNumber kDriftRate = new LoggedTunableNumber("Drive/DriftRate", DriveConstants.kDriftRate);

    @AutoLogOutput(key="Drive/CurrentState")
    private DriveState driveState = DriveState.TELEOP;

    @AutoLogOutput(key="Drive/HeadingGoal")
    private Rotation2d headingGoal = new Rotation2d();
    
    public Drive(SwerveModule[] modules, GyroHardware gyro){
        this.modules = modules;
        this.gyro = gyro;
        robotRotation = gyroInputs.yaw;

        swerveOdometry = new SwerveDriveOdometry(kinematics, getRobotRotation(), getModulePositions());
        swervePoseEstimator = new SwerveDrivePoseEstimator(kinematics, getRobotRotation(), getModulePositions(), new Pose2d());
        
        try {
            robotConfig = RobotConfig.fromGUISettings();
        } catch(Exception e) {
            e.printStackTrace();
        }

        generator = new SwerveSetpointGenerator(robotConfig, DriveConstants.kMaxRadiansPS);
        previousSetpoint = new SwerveSetpoint(
            new ChassisSpeeds(0, 0, 0), 
            new SwerveModuleState[] {
                new SwerveModuleState(), new SwerveModuleState(),
                new SwerveModuleState(), new SwerveModuleState()
            }, DriveFeedforwards.zeros(robotConfig.numModules));

        AutoBuilder.configure(
            this::getEstimatedPose,
            this::setPose, 
            this::getChassisSpeeds,
            (speeds) -> autonDesiredSpeeds = speeds, 
            new PPHolonomicDriveController(
                translationPathplannerConstants, 
                rotationPathplannerConstants), 
            robotConfig, 
            () -> DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red, 
            this);

        Pathfinding.setPathfinder(new LocalADStarAK());
        PathPlannerLogging.setLogActivePathCallback((activePath) -> Logger.recordOutput(
        "Drive/Odometry/Trajectory", activePath.toArray(new Pose2d[activePath.size()])));
        PathPlannerLogging.setLogTargetPoseCallback((targetPose) -> Logger.recordOutput(
        "Drive/Odometry/TrajectorySetpoint", targetPose));

        SmartDashboard.putData(field);

        headingController.setHeadingGoal(() -> headingGoal);
    }  

    @Override
    public void periodic(){
        gyro.updateInputs(gyroInputs);
        Logger.processInputs("Drive/Gyro", gyroInputs);

        for(SwerveModule module: modules){
            module.periodic();

            if(DriverStation.isDisabled()){
                module.stop();
            }
        }

        robotRotation = gyroInputs.yaw;

        swervePoseEstimator.update(robotRotation, getModulePositions());
        swerveOdometry.update(robotRotation, getModulePositions());

        field.setRobotPose(getEstimatedPose());

        ChassisSpeeds teleopSpeeds = teleopController.computeChassisSpeeds((getEstimatedPose().getRotation()), getChassisSpeeds());

        switch (driveState){
            case TELEOP:
                desiredSpeeds = teleopSpeeds;
                break;

            case SYS_ID:
                break;

            case SNIPER_UP:
                desiredSpeeds = new ChassisSpeeds(0.5, 0, 0);
                break;

            case SNIPER_DOWN:
                desiredSpeeds = new ChassisSpeeds(-0.5, 0, 0);
                break;

            case SNIPER_RIGHT:
                desiredSpeeds = new ChassisSpeeds(0, 0.5, 0);
                break;

            case SNIPER_LEFT:
                desiredSpeeds = new ChassisSpeeds(0, -0.5, 0);
                break;

            case AUTON:
                desiredSpeeds = new ChassisSpeeds(
                    autonDesiredSpeeds.vxMetersPerSecond, 
                    autonDesiredSpeeds.vyMetersPerSecond,
                    autonDesiredSpeeds.omegaRadiansPerSecond);
                break;

            case STOP:
                desiredSpeeds = null;
                for(SwerveModule module : modules) {
                    module.setDriveVolts(0.0);
                }
                break;
            default:
                desiredSpeeds = null;
                break;

        }

        if(desiredSpeeds != null){
            setSwerve(desiredSpeeds);
        }

        SmartDashboard.putData(field);
        
        Logger.recordOutput("Drive/desired speeds", desiredSpeeds);
    }  

    public void setDriveEnum(DriveState state){
        driveState = state;
    }

    // Changes the drive state which is logged in AS //
    public Command setDriveStateCommand(DriveState state){
        return Commands.runOnce(() -> setDriveEnum(state), this);
    }

    /**
     * ADD THIS 
     * @param state state to pass in
     * @return command
     */
    public Command setDriveStateCommandContinued(DriveState state){
        return new FunctionalCommand(
            () -> setDriveEnum(state), 
            () -> {}, 
            (interrupted) -> {}, 
            () -> false, 
            this);
    }

    /**
     * Runs characterization to find the gains of the drive motor 
     * @return the command that will be runned
     */
    public Command characterizeDriveMotors() {
        return setDriveStateCommand(DriveState.SYS_ID).andThen(
            SysIDCharacterization.runDriveSysIDTests( (voltage) -> {
                for (var module : modules) module.runLinearCharacterization(voltage);
        }, this));
    }

    public void setChassisSpeeds(ChassisSpeeds speeds){
        desiredSpeeds = speeds;
    }

    public void setVoltage(double voltage){
        for(SwerveModule mod : modules){
            mod.setDriveVolts(voltage);
        }
    }

    public void setSwerve(ChassisSpeeds speeds) {
        desiredSpeeds = discretize(speeds);
        // FOR LOGGING
        SwerveModuleState[] unoptimizedSetpointStates = kinematics.toSwerveModuleStates(desiredSpeeds);

        SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(desiredSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(unoptimizedSetpointStates, DriveConstants.kMaxLinearSpped);
        SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, DriveConstants.kMaxLinearSpped);

        SwerveModuleState[] optimizedSetpointStates = new SwerveModuleState[4];

        // 0.02 is the loop time of a periodic
        previousSetpoint = generator.generateSetpoint(previousSetpoint, desiredSpeeds, 0.02);

        for (int i = 0; i < 4; i++) {
            if(useGenerator) {
                setpointStates[i] = new SwerveModuleState(
                    previousSetpoint.moduleStates()[i].speedMetersPerSecond,
                    Math.abs(previousSetpoint.moduleStates()[i].speedMetersPerSecond / DriveConstants.kMaxLinearSpped) < 0.01 ?
                    modules[i].getCurrentState().angle : previousSetpoint.moduleStates()[i].angle);

                Logger.recordOutput("Drive/Swerve/Feedforward/"+i+"/Acceleration", previousSetpoint.feedforwards().accelerationsMPSSq()[i]);
                Logger.recordOutput("Drive/Swerve/Feedforward/"+i+"/Force", previousSetpoint.feedforwards().linearForcesNewtons()[i]);
                Logger.recordOutput("Drive/Swerve/Feedforward/"+i+"/Current", previousSetpoint.feedforwards().torqueCurrentsAmps()[i]);

                SwerveModuleState preOptimizedSetpointState = setpointStates[i];
                setpointStates[i].optimize(modules[i].getCurrentState().angle);
                boolean isModuleSpeedOptimized = isSpeedOptimized(preOptimizedSetpointState, setpointStates[i]);
                Logger.recordOutput("Drive/Swerve/Feedforward/"+i+"/optimalInvert", isModuleSpeedOptimized);

                optimizedSetpointStates[i].cosineScale(modules[i].getCurrentState().angle);

                optimizedSetpointStates[i] = modules[i].setSwerveStatewithAccel(
                    setpointStates[i], 
                    // Pathplanner does not handle inverting //
                    (isModuleSpeedOptimized) ? -1 * previousSetpoint.feedforwards().accelerationsMPSSq()[i] : previousSetpoint.feedforwards().accelerationsMPSSq()[i]);

                }
             
            
            else {
                // This all works and is fully tested //
                setpointStates[i] = new SwerveModuleState(
                    previousSetpoint.moduleStates()[i].speedMetersPerSecond,
                    Math.abs(previousSetpoint.moduleStates()[i].speedMetersPerSecond / DriveConstants.kMaxLinearSpped) < 0.01 ?
                    modules[i].getCurrentState().angle : previousSetpoint.moduleStates()[i].angle);

                setpointStates[i].optimize(modules[i].getCurrentState().angle);
                optimizedSetpointStates[i] = modules[i].setSwerveState(setpointStates[i]);
            }
        }
        
        Logger.recordOutput("Drive/Swerve/Setpoints", unoptimizedSetpointStates);
        Logger.recordOutput("Drive/Swerve/SetpointsOptimized", optimizedSetpointStates);
        Logger.recordOutput("Drive/Swerve/SetpointsChassisSpeeds", kinematics.toChassisSpeeds(optimizedSetpointStates));
    }

    public boolean isSpeedOptimized(SwerveModuleState state, SwerveModuleState optimizedState) {
        return state.speedMetersPerSecond != optimizedState.speedMetersPerSecond;
    }

    // The discretize function is used to break apart the fact that the values is not constant (its fed every 0.02) seconds //
    // This funciton is used to stop the chassis speeds from considered continuous and rather little slivers //
    // Looks at EN for L3 to learn more about it //
    private ChassisSpeeds discretize(ChassisSpeeds speeds) {
        double dt = 0.02;
        var desiredDeltaPose = new Pose2d(
            speeds.vxMetersPerSecond * dt,
            speeds.vyMetersPerSecond * dt,
            new Rotation2d(speeds.omegaRadiansPerSecond * dt * kDriftRate.get()));
        var twist = new Pose2d().log(desiredDeltaPose);

        return new ChassisSpeeds((twist.dx / dt), (twist.dy / dt), (speeds.omegaRadiansPerSecond));
    }

    public void stop(){
        setSwerve(new ChassisSpeeds());
    } 

    public void resetGyro() {
        robotRotation = Constants.kAlliance == Alliance.Blue ? Rotation2d.fromDegrees(0.0) : Rotation2d.fromDegrees(180.0);
        setPose(new Pose2d(getEstimatedPose().getTranslation(), robotRotation));
    }

    public void resetPose(){
        setPose(new Pose2d());
    }

    public void setPose(Pose2d pose){
        robotRotation = pose.getRotation();
        gyro.setYaw(pose.getRotation().getDegrees());
        swerveOdometry.resetPosition(robotRotation, getModulePositions(), pose);
        swervePoseEstimator.resetPosition(robotRotation, getModulePositions(), pose);
    }

    @AutoLogOutput(key = "Drive/Odometry/RobotRotation")
    public Rotation2d getRobotRotation(){
        return robotRotation;
    }

    @AutoLogOutput(key = "Drive/Odometry/GyroRotation")
    public Rotation2d getGyroRotation(){
        return gyroInputs.yaw;
    }

    @AutoLogOutput(key = "Drive/Odometry/PoseEstimate")
    public Pose2d getEstimatedPose(){
        return swervePoseEstimator.getEstimatedPosition();
    }

    @AutoLogOutput(key = "Drive/Odometry/DrivePose")
    public Pose2d getOdometryPose(){
        return swerveOdometry.getPoseMeters();
    }

    @AutoLogOutput(key = "Drive/Swerve/MeasuredPosistion")
    public SwerveModulePosition[] getModulePositions(){
        SwerveModulePosition[] posistions = new SwerveModulePosition[4];
        for(int i = 0; i < 4; i++){
            posistions[i] = modules[i].getCurrentPosistion();
        }

        return posistions;
    }

    @AutoLogOutput(key = "Drive/Swerve/MeasuredStates")
    public SwerveModuleState[] getModuleStates(){
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(int i = 0; i < 4; i++){
            states[i] = modules[i].getCurrentState();
        }

        return states;
    }

    @AutoLogOutput(key = "Drive/Odometry/CurrentChassisSpeeds")
    public ChassisSpeeds getChassisSpeeds() {
        return DriveConstants.kKinematics.toChassisSpeeds(getModuleStates());
    }

    @AutoLogOutput(key = "Drive/Odometry/DesiredChassisSpeeds")
    public ChassisSpeeds getDesiredChassisSpeeds() {
        return desiredSpeeds;
    }

    public void resetAllEncoders(){
        for(int i = 0; i < 4; i++){
            modules[i].resetAzimuthEncoder();
        }
    }

    public void acceptJoystickInputs(DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier thetaSupplier){
        teleopController.acceptJoystickInputs(xSupplier, ySupplier, thetaSupplier);
    }
}
