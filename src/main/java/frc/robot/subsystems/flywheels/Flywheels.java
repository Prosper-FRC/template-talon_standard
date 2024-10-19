package frc.robot.subsystems.flywheels;

import static frc.robot.subsystems.flywheels.FlywheelsConstants.kBottomConfig;
import static frc.robot.subsystems.flywheels.FlywheelsConstants.kTopConfig;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.AutoLogOutput;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.debugging.LoggedTunableNumber;
import frc.robot.utils.debugging.SysIDCharacterization;
import frc.robot.utils.math.LinearProfile;

import org.littletonrobotics.junction.Logger;

public class Flywheels extends SubsystemBase {
    @AutoLog
    public static class FlywheelInputs {
        public boolean isConnected = true;
        public double velocityMPS = 0.0;
        public double appliedVolts = 0.0;
        public double motorVolts = 0.0;
        public double[] statorCurrentAmps = {0.0};
        public double[] supplyCurrentAmps = {0.0};
        public double[] temperatureCelsius = {0.0};
    }

    public static enum FlywheelSetpoint {

        SHOOT(() -> 20.0, () ->  20.0),
        FEED_SHOOT(()-> 17.5, ()-> 17.5),
        IDLE(() -> 0.0, () -> 0.0),
        IDLE_RUNNING(() -> 9.0, () -> 9.0),
        DUMP(() -> 9.0, () -> 9.0),
        /* SPECIAL CASE */ 
        AMP(() -> 0.0, () -> 0.0),
        STOP(() -> 0.0, () -> 0.0);

        private DoubleSupplier topVelocityMPS;
        private DoubleSupplier bottomVelocityMPS;

        private FlywheelSetpoint(DoubleSupplier topVelocityMPS, DoubleSupplier bottomVelocityMPS) {
            this.topVelocityMPS = topVelocityMPS;
            this.bottomVelocityMPS = bottomVelocityMPS;
        }

        public DoubleSupplier getTopVelocityMPS() {
            return topVelocityMPS;
        }

        public DoubleSupplier getBottomVelocityMPS() {
            return bottomVelocityMPS;
        }
    }

    public static LoggedTunableNumber kP = 
        new LoggedTunableNumber("Shooter/Flywheel/kP", FlywheelsConstants.kControllerConfig.kP());
    public static LoggedTunableNumber kS = 
        new LoggedTunableNumber("Shooter/Flywheel/kS", FlywheelsConstants.kControllerConfig.kS());
    public static LoggedTunableNumber kV = 
        new LoggedTunableNumber("Shooter/Flywheel/kV", FlywheelsConstants.kControllerConfig.kV());
    public static LoggedTunableNumber kA = 
        new LoggedTunableNumber("Shooter/Flywheel/kA", FlywheelsConstants.kControllerConfig.kA());
    public static LoggedTunableNumber kMaxAccel = 
        new LoggedTunableNumber("Shooter/Flywheel/kMaxAccel", FlywheelsConstants.kMaxAccelerationMPSS);

    private FlywheelsKrakenHardware topFlywheel;
    private FlywheelInputsAutoLogged topInputs = new FlywheelInputsAutoLogged();
    private LinearProfile topProfile = new LinearProfile(kMaxAccel.get(), 0.02);

    private FlywheelsKrakenHardware bottomFlywheel;
    private FlywheelInputsAutoLogged bottomInputs = new FlywheelInputsAutoLogged();
    private LinearProfile bottomProfile = new LinearProfile(kMaxAccel.get(), 0.02);

    private LoggedTunableNumber ampVolt1 = new LoggedTunableNumber("Intake/AmpVolt1", 2.45);
    private LoggedTunableNumber ampVolt2 = new LoggedTunableNumber("Intake/AmpVolt2", 0);

    @AutoLogOutput
    private FlywheelSetpoint goal = FlywheelSetpoint.IDLE;

    public Flywheels() {
        topFlywheel = new FlywheelsKrakenHardware(kTopConfig);
        bottomFlywheel = new FlywheelsKrakenHardware(kBottomConfig);
        setGoal(FlywheelSetpoint.IDLE);
    }

    public FlywheelSetpoint getGoal() {
        return goal;
    }

    @Override
    public void periodic() {
        topFlywheel.updateInputs(topInputs);
        Logger.processInputs("Flywheels/Top", topInputs);

        bottomFlywheel.updateInputs(bottomInputs);
        Logger.processInputs("Flywheels/Bottom", bottomInputs);

        if(goal != null) {
            if(!goal.equals(FlywheelSetpoint.AMP)) {
                double topSetpoint = topProfile.calculateSetpoint();
                double topAcceleration = topProfile.getCurrentAcceleration();
                Logger.recordOutput("Flywheels/TopGoal", goal.getTopVelocityMPS().getAsDouble());
                Logger.recordOutput("Flywheels/TopSetpoint", topSetpoint);
                Logger.recordOutput("Flywheels/TopAcceeleration", topAcceleration);

                double bottomSetpoint = bottomProfile.calculateSetpoint();
                double bottomAcceleration = bottomProfile.getCurrentAcceleration();
                Logger.recordOutput("Flywheels/BottomGoal", goal.getBottomVelocityMPS().getAsDouble());
                Logger.recordOutput("Flywheels/BottomSetpoint", bottomSetpoint);
                Logger.recordOutput("Flywheels/BottomAcceeleration", bottomAcceleration);

                topFlywheel.setVelocity(topSetpoint, topAcceleration);
                bottomFlywheel.setVelocity(bottomSetpoint, bottomAcceleration);
            } else {
                topFlywheel.setVolts(ampVolt2.get());
                bottomFlywheel.setVolts(ampVolt1.get());
            }
        }

        LoggedTunableNumber.ifChanged(hashCode(), () -> {
            topFlywheel.setPFF(kP.get(), kS.get(), kV.get(), kA.get());
            bottomFlywheel.setPFF(kP.get(), kS.get(), kV.get(), kA.get());
        }, kP, kS, kV, kA);

        if(DriverStation.isDisabled()) stopFlywheels();
    }

    public void setGoal(FlywheelSetpoint goal) {
        this.goal = goal;
        if(this.goal != null) {
            topProfile.setGoal(goal.topVelocityMPS.getAsDouble(), topInputs.velocityMPS);
            bottomProfile.setGoal(goal.bottomVelocityMPS.getAsDouble(), bottomInputs.velocityMPS);
        }
    }

    public Command setGoalCommand(FlywheelSetpoint goal) {
        return Commands.runOnce(() -> setGoal(goal), this);
    }

    public void setTopVolts(double volts) {
        topFlywheel.setVolts(volts);
    }

    public void setBottomVolts(double volts) {
        bottomFlywheel.setVolts(volts);
    }

    public void stopFlywheels() {
        topFlywheel.setVolts(0.0);
        bottomFlywheel.setVolts(0.0);
    }

    @AutoLogOutput(key="Flywheels/InTolerance")
    public boolean inTolerance() {
        return (topProfile.getGoal() - topInputs.velocityMPS) < 1.0
            && (bottomProfile.getGoal() - bottomInputs.velocityMPS) < 1.0;
    }

    public Command characterizeShooter() {
        return SysIDCharacterization.runShooterSysIDTests((v) -> {
            setGoal(null);
            topFlywheel.setVolts(v);
            bottomFlywheel.setVolts(v);
        }, this);
    }
}
