package frc.robot.pneumatics;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Pneumatics extends SubsystemBase {

  // For this implementation we are going to have two solenoids for Susan's intake //
  // Look at slides to see Susan intake //
  private DoubleSolenoid rightSolenoid;
  private DoubleSolenoid leftSolenoid;

  // We dont use the compressor but it needs to be instantiated anways //
  // This is to make sure that the compressor holds air for the solenoids //
  private Compressor compressor;

  public Pneumatics() {

    // Use the PneumaticsConstants class to get the forward and reverse channel of the Double Solenoid //
    // Also, if you are using a CTRE pneumatics hub change 'REVPH' to 'CTREPM' //
    // Module number is default going to be '1' //
    // Read here to learn more about it -> https://docs.wpilib.org/en/stable/docs/software/hardware-apis/pneumatics/index.html //
    rightSolenoid =
        new DoubleSolenoid(
          PnematicsConstants.moduleID,
          PneumaticsModuleType.REVPH,
          PnematicsConstants.rightSolenoidPorts[0],
          PnematicsConstants.rightSolenoidPorts[1]);

    leftSolenoid =
        new DoubleSolenoid(
          PnematicsConstants.moduleID,
          PneumaticsModuleType.REVPH,
          PnematicsConstants.leftSolenoidPorts[0],
          PnematicsConstants.leftSolenoidPorts[1]);

    // Extantiate the compressor object so that air flow starts //
    compressor = new Compressor(PneumaticsModuleType.REVPH);

    // Setting an intial state to the solenoids make sure that the 'toggle' method works //
    // Toggle can only work is the solenoid is set to 'reverse' or 'forward' instead of 'off' //
    leftSolenoid.set(Value.kReverse);
    rightSolenoid.set(Value.kReverse);

  }

  @Override
  public void periodic() {

    // Log the state of the solenoids //
    Logger.recordOutput("/Pneumatics/right/Value", getRightState());
    Logger.recordOutput("/Pneumatics/left/Value", getLeftState());
  }

  // This command will be schduled when a button is clicked //  /
  // There are different typees of command, this is an instantCommadn which takes a method and lets us schedule it //
  public Command toggleSolenoids() {
    return new InstantCommand(
        () -> {
          // Toggle changes the state of the solenoid //
          // Forward -> Retract //
          // Retracted -> Forward //
          rightSolenoid.toggle();
          leftSolenoid.toggle();
        },
        this);
  }

  // Grab these values for telemetry //
  public String getRightState() {
    // State of the solenoid //
    DoubleSolenoid.Value result = rightSolenoid.get();

    // Three different possiblities for the result (Forward, Reverse, and Off) //
    if(result.equals(DoubleSolenoid.Value.kForward)){
      return "Forward";
    }

    else if(result.equals(DoubleSolenoid.Value.kReverse)){
      return "Reverse";
    }

    else{
      return "Off";
    }
  }

  public String getLeftState() {
    // State of the solenoid //
    DoubleSolenoid.Value result = leftSolenoid.get();

    // Three different possiblities for the result (Forward, Reverse, and Off) //
    if(result.equals(DoubleSolenoid.Value.kForward)){
      return "Forward";
    }

    else if(result.equals(DoubleSolenoid.Value.kReverse)){
      return "Reverse";
    }

    else{
      return "Off";
    }
  }
}
