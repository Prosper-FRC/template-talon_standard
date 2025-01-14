package frc.robot.drive;

import org.littletonrobotics.junction.AutoLog;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import frc.robot.Constants;

public class GyroHardware {

    private Pigeon2 gyroscope;
    private StatusSignal<Angle> yaw;

    @AutoLog
    public static class GyroInputs {
        public boolean gyroConnected = true;
        public Rotation2d yaw = new Rotation2d();
    }
    
    public GyroHardware(){
        gyroscope = new Pigeon2(DriveConstants.kGyroPort, Constants.kCanbusName);
        gyroscope.getConfigurator().apply(new Pigeon2Configuration());
        gyroscope.setYaw(0);

        yaw = gyroscope.getYaw();
        BaseStatusSignal.setUpdateFrequencyForAll(50.0, yaw);
        gyroscope.optimizeBusUtilization();
    }

    public void updateInputs(GyroInputs inputs){
        inputs.gyroConnected = BaseStatusSignal.refreshAll(yaw).isOK();
        inputs.yaw = Rotation2d.fromDegrees(yaw.getValueAsDouble());
    }

    public void setYaw(double yaw){
        gyroscope.setYaw(yaw);
    }
}
