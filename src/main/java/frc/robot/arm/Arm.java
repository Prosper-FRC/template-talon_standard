package frc.robot.subsystems.arm;

import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.utils.debugging.LoggedTunableNumber;
// import frc.robot.utils.math.EqualsUtil;

import static frc.robot.subsystems.arm.ArmConstants.*;

public class Arm extends SubsystemBase {
    private static Rotation2d lastRotationGoal = Rotation2d.fromDegrees(6.0);

    public static enum ArmGoal {
        REMOVE_NOTE(() -> Rotation2d.fromDegrees(85.0)),
        SPEAKER(() -> Rotation2d.fromDegrees(50.0)),
        SPEAKER_SIDE(() -> Rotation2d.fromDegrees(51.0)),
        FEED(() -> Rotation2d.fromDegrees(65.0)),
        AMP(() -> Rotation2d.fromDegrees(50.0 - 0.5)),
        PODIUM(() -> Rotation2d.fromDegrees(35.0)),
        WING(() -> Rotation2d.fromDegrees(25.5)),
        INTAKE(() -> Rotation2d.fromDegrees(20.0)),
        IDLE(() -> Rotation2d.fromDegrees(8.0)),
        AUTO_AIM(() -> AutoShoot.getInstance().getShotAngle()),
        // Special cases handled in periodic
        MANUAL_UP(() -> Rotation2d.fromDegrees(6.0)),
        MANUAL_DOWN(() -> Rotation2d.fromDegrees(6.0)),
        STAY(() -> lastRotationGoal),
        DEBUGGING(() -> Rotation2d.fromDegrees(6.0));

        private Supplier<Rotation2d> rotSupplier;

        private ArmGoal(Supplier<Rotation2d> rotSupplier) {
            this.rotSupplier = rotSupplier;
        }

        public Supplier<Rotation2d> getGoal() {
            return rotSupplier;
        }
    }

    private static final LoggedTunableNumber kP = new LoggedTunableNumber(
        "Arm/kP", kControllerConfig.kP());
    private static final LoggedTunableNumber kD = new LoggedTunableNumber(
        "Arm/kD", kControllerConfig.kD());
    private static final LoggedTunableNumber kMaxV = new LoggedTunableNumber(
        "Arm/kMaxV", kControllerConfig.kMaxV());
    private static final LoggedTunableNumber kMaxA = new LoggedTunableNumber(
        "Arm/kMaxA", kControllerConfig.kMaxA());
    private static final LoggedTunableNumber kS = new LoggedTunableNumber(
        "Arm/kS", kControllerConfig.kS());
    private static final LoggedTunableNumber kG = new LoggedTunableNumber(
        "Arm/kG", kControllerConfig.kG());
    private static final LoggedTunableNumber kV = new LoggedTunableNumber(
        "Arm/kV", kControllerConfig.kV());

    private static final LoggedTunableNumber kManualSpeed = 
        new LoggedTunableNumber("Arm/ManualSpeedDegrees", kAngularPerSecond.getDegrees());

    private static final LoggedTunableNumber kDebugginGoal = 
        new LoggedTunableNumber("Arm/DebuggingGoalDegrees", 10.0);

    @AutoLogOutput(key = "Arm/Goal")
    private ArmGoal goal = ArmGoal.IDLE;

    private ProfiledPIDController armPID;
    private ArmFeedforward armFF;

    @AutoLogOutput(key = "Arm/RobotPivotAngle")
    private Rotation2d robotPivotAngle = new Rotation2d();

    private ShooterVisualizer armVisualizer = new ShooterVisualizer(Rotation2d.fromDegrees(6.0));

    public Arm(ArmIO io) {
        this.io = io;
        armPID = new ProfiledPIDController(
            kP.get(), 0.0, kD.get(), 
            new Constraints(kMaxV.get(), kMaxA.get()));
        armPID.setTolerance(0.0);
        armPID.enableContinuousInput(0, 360);

        armFF = new ArmFeedforward(kS.get(), kG.get(), kV.get(), 0.03);

        setGoal(null);
    }

    public void periodic() {
        if(armInputs.dutycycleFrequency == 957) {
            robotPivotAngle = armInputs.absoluteRotation;
        } else if(armInputs.dutycycleFrequency == 958) {
            robotPivotAngle = armInputs.absoluteRotation.plus(Rotation2d.fromDegrees(0.35));
        }

        LoggedTunableNumber.ifChanged(hashCode(), () -> {
            armPID.setP(kP.get());
            armPID.setD(kD.get());
            armPID.setConstraints(new Constraints(kMaxV.get(), kMaxA.get()));
        }, kP, kD, kMaxV, kMaxA);

        LoggedTunableNumber.ifChanged(hashCode(), () -> {
            armFF = new ArmFeedforward(0.0, kG.get(), kV.get());
        }, kG, kV);

        double volts = 0;
        if(goal != null) {

            Rotation2d rotationGoal;
            switch(goal) {
                case MANUAL_UP:
                    rotationGoal = robotPivotAngle.plus(
                        Rotation2d.fromDegrees(kManualSpeed.get() * 0.02));
                        break;
                case MANUAL_DOWN:
                    rotationGoal = robotPivotAngle.minus(
                        Rotation2d.fromDegrees(kManualSpeed.get() * 0.02));
                        break;
                case DEBUGGING:
                    rotationGoal = Rotation2d.fromDegrees(kDebugginGoal.get());
                    break;
                // Blubber
                case AUTO_AIM:
                case IDLE:
                case INTAKE:
                case WING:
                case PODIUM:
                case AMP:
                case FEED:
                case SPEAKER:
                    rotationGoal = goal.getGoal().get();
                    break;
                // Had to satisfy compiler
                default:
                    rotationGoal = goal.getGoal().get();
            }

        }

        armVisualizer.updateShooterAngle(armInputs.absoluteRotation);

        if(DriverStation.isDisabled() || 
        (.getDegrees() > kMaxAngle.getDegrees() && volts > 0) || 
        (.getDegrees() < kMinAngle.getDegrees() && volts < 0)) {
            stopMotor();
        }
    }

    public void setGoal(ArmGoal goal) {
        this.goal = goal;
        resetPIDController();
    }

    public Command setGoalCommand(ArmGoal goal) {
        return Commands.runOnce(() -> setGoal(goal), this);
    }

    public void resetPIDController() {
        if(goal != null ) {
            // if(Math.abs(armInputs.absoluteRotation.minus(this.goal.getGoal().get()).getDegrees()) > 2.0) {
                armPID.reset(
                    armInputs.absoluteRotation.getDegrees(), 
                    0.0);
                    // armInputs.angularSpeedPerSecond.getDegrees());
            // }
        }
    }

    @AutoLogOutput
    public boolean inTolerance() {
        return Math.abs(armPID.getGoal().position - robotPivotAngle.getDegrees()) < 0.5;
    }

    @AutoLogOutput(key = "Arm/AtGoal")
    public boolean atGoal() {
        return armPID.atGoal();
    }

    @AutoLogOutput(key = "Arm/RotationGoal")
    public Rotation2d getGoal() {
        return Rotation2d.fromDegrees(armPID.getGoal().position);
    }

    @AutoLogOutput(key = "Arm/RotationSetpoint")
    public Rotation2d getSetpoint() {
        return Rotation2d.fromDegrees(armPID.getSetpoint().position);
    }

    public void setVolts(double volts) {
        io.setVolts(volts);
    }

    public void stopMotor() {
        io.setVolts(0.0);
    }

    public ArmGoal getArmGoal() {
        return goal;
    }
}