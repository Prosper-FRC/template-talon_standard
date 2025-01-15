package frc.robot.drive;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.debugging.LoggedTunableNumber;

public class Module extends SubsystemBase {

    public static final LoggedTunableNumber driveP = new LoggedTunableNumber("Module/Drive/kP", DriveConstants.kModuleControllerConfigs.driveController().getP());
    public static final LoggedTunableNumber driveD = new LoggedTunableNumber("Module/Drive/kD", DriveConstants.kModuleControllerConfigs.driveController().getD());
    public static final LoggedTunableNumber driveS = new LoggedTunableNumber("Module/Drive/kS", DriveConstants.kModuleControllerConfigs.driveFF().getKs());
    public static final LoggedTunableNumber driveV = new LoggedTunableNumber("Module/Drive/kV", DriveConstants.kModuleControllerConfigs.driveFF().getKv());
    public static final LoggedTunableNumber driveA = new LoggedTunableNumber("Modulkeye/Drive/kA", DriveConstants.kModuleControllerConfigs.driveFF().getKa());

    public static final LoggedTunableNumber azimuthP = new LoggedTunableNumber("Module/Azimuth/kP", DriveConstants.kModuleControllerConfigs.azimuthController().getP());
    public static final LoggedTunableNumber azimuthD = new LoggedTunableNumber("Module/Azimuth/kD", DriveConstants.kModuleControllerConfigs.azimuthController().getD());
    public static final LoggedTunableNumber azimuthS = new LoggedTunableNumber("Module/Azimuth/kS", DriveConstants.kModuleControllerConfigs.azimuthFF().getKa());

    // Different set points for the azimuth and drive motors //
    private Double velocitySetpointMPS = null;
    private Rotation2d azimuthSetpointAngle = null;
    private Double accelerationSetpointMPSS = null;

    // Current state and current posistion are updated periodically //
    private SwerveModuleState currentState = new SwerveModuleState();
    private SwerveModulePosition currentPosition = new SwerveModulePosition();

    // IO layer along with the autologged inputs that are processed in this file //
    private ModuleIO io;
    private SwerveModuleInputsAutoLogged inputs = new SwerveModuleInputsAutoLogged();

    // This drive ff was added for when acceleration + velocity needs to be set //
    // This allows for the calculation of the feedforward value between the velocity and acceleration //
    private SimpleMotorFeedforward driveFeedforward = new SimpleMotorFeedforward(0, 0, 0);

    private String nameKey;

    public Module(String key, ModuleIO config){
        this.io = (config);
        nameKey = "Module/" + key;
    }

    public void periodic(){
        io.updateInputs(inputs);
        Logger.processInputs("Drive/"+ nameKey, inputs);

        currentState = new SwerveModuleState(inputs.driveVelocityMPS, inputs.azimuthPosistion);
        currentPosition = new SwerveModulePosition(inputs.drivePosistionM, inputs.azimuthPosistion);

        // Checks if setpoint is existing before setting a demand // 
        if(velocitySetpointMPS != null){

            if(accelerationSetpointMPSS != null){
                double feedforward = driveFeedforward.calculate(velocitySetpointMPS, accelerationSetpointMPSS);
                io.setDriveVelocity(velocitySetpointMPS, feedforward);
            }

            else{
                io.setDriveVelocity(velocitySetpointMPS, 0);
            }
        }

        if(azimuthSetpointAngle != null){
            io.setAzimuthPosition(azimuthSetpointAngle);
        }

        LoggedTunableNumber.ifChanged(
            hashCode(), () -> {
                io.setDriveGains(driveP.get(), 0.0, driveD.get(), driveS.get(), driveV.get(), driveA.get());
            }, driveP, driveD);

        LoggedTunableNumber.ifChanged(
            hashCode(), () -> {
                driveFeedforward = new SimpleMotorFeedforward(driveS.get(), driveV.get(), driveA.get());
            }, driveS, driveV, driveA);

        LoggedTunableNumber.ifChanged(
            hashCode(), () -> {
                io.setAzimuthPID(azimuthP.get(), 0.0, azimuthD.get());
            }, azimuthP, azimuthD);


    }

    /**
     * Reset the posistion of the azimuth encoder
     */
    public void resetAzimuthEncoder(){
        io.resetAzimuthEncoder();
    }

    /**
     * Run characterization for drive motor gains. Dont use kA from Sys ID
     * Bind this to a button and let it run
     * @param inputVolts volts fed into the motor
     */
    public void runLinearCharacterization(double inputVolts) {
        setAzimuthPosistion(Rotation2d.fromRotations(0));
        setDriveVelocity(null);
        setDriveVolts(inputVolts);
    }

    /**
     * Set the desired velocity/posistion respectivly to the drive and azimuth with no acceleration
     * @param state desired state (posistion + velocity)
     * @return set state
     */
    public SwerveModuleState setSwerveState(SwerveModuleState state){
        setDriveAcceleration(null);
        setDriveVelocity(state.speedMetersPerSecond);
        setAzimuthPosistion(state.angle);
        return new SwerveModuleState(velocitySetpointMPS, azimuthSetpointAngle);
    }

    /**
     * Set the desired velocity/posistion respectivly to the drive and azimuth with acceleration
     * @param state desired state (posistion + velocity)
     * @param accel desired acceleration (m/s^2)
     * @return set state
     */
    public SwerveModuleState setSwerveStatewithAccel(SwerveModuleState state, double accel){
        setDriveAcceleration(accelerationSetpointMPSS);
        setDriveVelocity(state.speedMetersPerSecond);
        setAzimuthPosistion(state.angle);
        return new SwerveModuleState(velocitySetpointMPS, azimuthSetpointAngle);
    }

    /**
     * Set the desired posistion of the azimuth
     * @param state desired state (posistion + velocity), velocity will not be used
     * @return set state
     */
    public SwerveModuleState setAzimuthPosistion(SwerveModuleState state){
        setAzimuthPosistion(state.angle);
        setDriveVelocity(null);
        setDriveAcceleration(null);
        return new SwerveModuleState(0, azimuthSetpointAngle);
    }

    /**
     * Returns the velocity/posistion of the drive and azimuth respectively
     * @return current state of the swerve module
     */
    public SwerveModuleState getCurrentState(){
        return currentState;
    }

    /**
     * Returnds the distance traveled/posistion of the drive and azimuth respectively
     * @return current position of the swerve module
     */
    public SwerveModulePosition getCurrentPosistion(){
        return currentPosition;
    }

    /**
     * Set velocity demand to the drive motor
     * @param velocityDemand (units : m/s)
     */
    public void setDriveVelocity(Double velocityDemand){
        velocitySetpointMPS = velocityDemand;
    }

    /**
     * Set acceleration demand to the drive motor
     * @param accelDemand (units : m/s^2)
     */
    public void setDriveAcceleration(Double accelDemand){
        accelerationSetpointMPSS = accelDemand;
    }

    /**
     * Set voltage to the drive motor
     * @param volts (range: 0-12)
     */
    public void setDriveVolts(Double volts){
        io.setDriveVolts(volts);
    }

    /**
     * Set posistion to the azimuth motor
     * @param positionDemand (units: rotations)
     */
    public void setAzimuthPosistion(Rotation2d posistionDemand){
        azimuthSetpointAngle = posistionDemand;
    }

    /**
     * Set voltage to the azimuth motor
     * @param volts (range: 0-12)
     */
    public void setAzimuthVolts(Double volts){
        io.setAzimuthVolts(volts);
    }

    /**
     * Get the autologged inputs
     * @return inputs
     */
    public SwerveModuleInputsAutoLogged getInputs(){
        return inputs;
    }

    /**
     * Stop swerve module
     */
    public void stop(){
        io.setAzimuthVolts(0);
        io.setDriveVolts(0);
    }

}
