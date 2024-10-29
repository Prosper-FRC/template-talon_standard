package frc.robot.pneumatics;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Pneumatics extends SubsystemBase{

  private DoubleSolenoid rightSolenoid;
  private DoubleSolenoid leftSolenoid;
  private Compressor compressor;

  public Pneumatics() {

    rightSolenoid = new DoubleSolenoid(
        PneumaticsModuleType.REVPH, 
        PnematicsConstants.rightSolenoidPorts[0], 
        PnematicsConstants.rightSolenoidPorts[1]
    );
    
    leftSolenoid = new DoubleSolenoid(
        PneumaticsModuleType.REVPH, 
        PnematicsConstants.leftSolenoidPorts[0], 
        PnematicsConstants.leftSolenoidPorts[1]
    );
    
    compressor = new Compressor(PneumaticsModuleType.REVPH);
  }

 @Override
  public void periodic(){
    Logger.recordOutput("/Pneumatics/right/Value", returnRightPneumatic());
    Logger.recordOutput("/Pneumatics/left/Value", returnLeftPneumatic());
  }

  public Command extendPneumatics(){
    return new InstantCommand(() -> {setPneumatics(Value.kForward, Value.kForward);}, this);
  }

  public Command retractPneumatics(){
    return new InstantCommand(() -> {setPneumatics(Value.kReverse, Value.kReverse);}, this);
  }

  public void setPneumatics(Value right, Value left){
    rightSolenoid.set(right);
    leftSolenoid.set(left);
  }

  public Value returnRightPneumatic(){
    return rightSolenoid.get();
  }

  public Value returnLeftPneumatic(){
    return leftSolenoid.get();
  }

}
