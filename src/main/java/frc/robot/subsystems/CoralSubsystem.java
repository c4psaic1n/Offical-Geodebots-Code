package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.CoralSubsystemConstants;
import frc.robot.Constants.CoralSubsystemConstants.ArmSetpoints;
import frc.robot.Constants.CoralSubsystemConstants.ElevatorSetpoints;
import frc.robot.Constants.CoralSubsystemConstants.IntakeSetpoints;


public class CoralSubsystem extends SubsystemBase {
   //Subsystem-wide setpoints
  public enum Setpoint {
    kFeederStation,
    kLevel1,
    kLevel2,
    kLevel3,
    kLevel4;
  }
  // Initialize arm SPARK. We will use MAXMotion position control for the arm, so we also need to
  // initialize the closed loop controller and encoder.
  private SparkMax armMotor = new SparkMax(CoralSubsystemConstants.kArmMotorCanId, MotorType.kBrushless);
  private SparkClosedLoopController armController = armMotor.getClosedLoopController();
  private RelativeEncoder armEncoder = armMotor.getEncoder();

  // Initialize elevator SPARK. We will use MAXMotion position control for the elevator, so we also
  // need to initialize the closed loop controller and encoder.
  private SparkFlex elevatorMotor =new SparkFlex(CoralSubsystemConstants.kElevatorMotorCanId, MotorType.kBrushless);
  private SparkClosedLoopController elevatorClosedLoopController = elevatorMotor.getClosedLoopController();
  private RelativeEncoder elevatorEncoder = elevatorMotor.getEncoder();
 
  // Initialize intake SPARK. We will use open loop control for this so we don't need a closed loop
  // controller like above.
  private SparkMax intakeMotor =
      new SparkMax(CoralSubsystemConstants.kIntakeMotorCanId, MotorType.kBrushless);




  // Member variables for subsystem state management
  private boolean wasResetByButton = false;
  private boolean wasResetByLimit = false;
  private double armCurrentTarget = ArmSetpoints.kFeederStation;
  private double elevatorCurrentTarget = ElevatorSetpoints.kFeederStation;
  private Setpoint currentSetpoint = Setpoint.kFeederStation;
  public boolean isRunningElevator = false;




  public CoralSubsystem() {
   
      /* Apply the appropriate configurations to the SPARKs.
     *
     * kResetSafeParameters is used to get the SPARK to a known state. This
     * is useful in case the SPARK is replaced.
     *
     * kPersistParameters is used to ensure the configuration is not lost when
     * the SPARK loses power. This is useful for power cycles that may occur
     * mid-operation.
     */
    armMotor.configure(
        Configs.CoralSubsystem.armConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    elevatorMotor.configure(
        Configs.CoralSubsystem.elevatorConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    intakeMotor.configure(
        Configs.CoralSubsystem.intakeConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);






    // Zero arm and elevator encoders on initialization
    armEncoder.setPosition(0);
    elevatorEncoder.setPosition(0);



  }

  /**
   * Drive the arm and elevator motors to their respective setpoints. This will use MAXMotion
   * position control which will allow for a smooth acceleration and deceleration to the mechanisms'
   * setpoints.
   */
  private void moveToSetpoint() {
    armController.setReference(armCurrentTarget, ControlType.kMAXMotionPositionControl);
    elevatorClosedLoopController.setReference(elevatorCurrentTarget, ControlType.kMAXMotionPositionControl);
  }




  // Zero the elevator encoder when the limit switch is pressed. 
  private void zeroElevatorOnLimitSwitch() {
    if (!wasResetByLimit && elevatorMotor.getReverseLimitSwitch().isPressed()) {
      // Zero the encoder only when the limit switch is switches from "unpressed" to "pressed" to
      // prevent constant zeroing while pressed
      elevatorEncoder.setPosition(0);
      wasResetByLimit = true;
    } else if (!elevatorMotor.getReverseLimitSwitch().isPressed()) {
      wasResetByLimit = false;
    }
  }

  // Zero the arm and elevator encoders when the user button is pressed on the roboRIO. *
  private void zeroOnUserButton() {
    if (!wasResetByButton && RobotController.getUserButton()) {
      // Zero the encoders only when button switches from "unpressed" to "pressed" to prevent
      // constant zeroing while pressed
      wasResetByButton = true;
      armEncoder.setPosition(0);
      elevatorEncoder.setPosition(0);
    } else if (!RobotController.getUserButton()) {
      wasResetByButton = false;
    }
  }





  // Set the intake motor power in the range of [-1, 1]. 
  private void setIntakePower(double power) {
    intakeMotor.set(power);
  }


  


  /**
   * Command to set the subsystem setpoint. This will set the arm and elevator to their predefined
   * positions for the given setpoint.
   */
  public Command setSetpointCommand(Setpoint setpoint) {
    return this.runOnce(
        () -> {
          switch (setpoint) {
            case kFeederStation:
              elevatorCurrentTarget = ElevatorSetpoints.kFeederStation;
              armCurrentTarget = ArmSetpoints.kFeederStation;
              break;

              
            case kLevel1:
              elevatorCurrentTarget = ElevatorSetpoints.kLevel1;
              armCurrentTarget = ArmSetpoints.kLevel1;
              break;
            case kLevel2:
              elevatorCurrentTarget = ElevatorSetpoints.kLevel2;
              armCurrentTarget = ArmSetpoints.kLevel2;
              break;
            case kLevel3:
              elevatorCurrentTarget = ElevatorSetpoints.kLevel3;
              armCurrentTarget = ArmSetpoints.kLevel3;
              break;
            case kLevel4:
              elevatorCurrentTarget = ElevatorSetpoints.kLevel4;
              armCurrentTarget = ArmSetpoints.kLevel4;
              break;
          }
        });
  }



  private void setSetpoint(Setpoint setpoint) {
    switch (setpoint) {
        case kFeederStation:
            elevatorCurrentTarget = ElevatorSetpoints.kFeederStation;
            armCurrentTarget = ArmSetpoints.kFeederStation;
            break;
        case kLevel1:
            elevatorCurrentTarget = ElevatorSetpoints.kLevel1;
            armCurrentTarget = ArmSetpoints.kLevel1;
            break;
        case kLevel2:
            elevatorCurrentTarget = ElevatorSetpoints.kLevel2;
            armCurrentTarget = ArmSetpoints.kLevel2;
            break;
        case kLevel3:
            elevatorCurrentTarget = ElevatorSetpoints.kLevel3;
            armCurrentTarget = ArmSetpoints.kLevel3;
            break;
        case kLevel4:
            elevatorCurrentTarget = ElevatorSetpoints.kLevel4;
            armCurrentTarget = ArmSetpoints.kLevel4;
            break;
    }
}



  public Command incrementSetpointCommand() {
    return this.runOnce(
        () -> {
            switch (currentSetpoint) {
                case kFeederStation:
                    currentSetpoint = Setpoint.kLevel1;
                    break;
                case kLevel1:
                    currentSetpoint = Setpoint.kLevel2;
                    break;
                case kLevel2:
                    currentSetpoint = Setpoint.kLevel3;
                    break;
                case kLevel3:
                    currentSetpoint = Setpoint.kLevel4;
                    break;
                case kLevel4:
                    // Already at the highest setpoint, do nothing or wrap around
                    break;
            }
            setSetpoint(currentSetpoint);
        });
}


public Command decrementSetpointCommand() {
  return this.runOnce(
      () -> {
          switch (currentSetpoint) {
              case kFeederStation:
                  // Already at the lowest setpoint, do nothing or wrap around
                  break;
              case kLevel1:
                  currentSetpoint = Setpoint.kFeederStation;
                  break;
              case kLevel2:
                  currentSetpoint = Setpoint.kLevel1;
                  break;
              case kLevel3:
                  currentSetpoint = Setpoint.kLevel2;
                  break;
              case kLevel4:
                  currentSetpoint = Setpoint.kLevel3;
                  break;
          }
          setSetpoint(currentSetpoint);
      });
}


  /**
   * Command to run the intake motor. When the command is interrupted, e.g. the button is released,
   * the motor will stop.
   */

  public Command runIntakeCommand() {
    return this.startEnd(
        () -> this.setIntakePower(IntakeSetpoints.kForward), () -> this.setIntakePower(0.0));
  }

  /**
   * Command to reverses the intake motor. When the command is interrupted, e.g. the button is
   * released, the motor will stop.
   */
  public Command reverseIntakeCommand() {
    return this.startEnd(
        () -> this.setIntakePower(IntakeSetpoints.kReverse), () -> this.setIntakePower(0.0));
  }

  public Command setElevatorSpeed(double speed) {
    isRunningElevator = true;
    return this.run(() -> elevatorMotor.set(speed));
  }
  
  public Command stopElevator() {
    isRunningElevator = false;
    return this.run(() -> elevatorMotor.set(0.01));
  }


  @Override
  public void periodic() {
    moveToSetpoint();
    zeroElevatorOnLimitSwitch();
    zeroOnUserButton();

    /*if(!isRunningElevator){
      elevatorMotor.set(0.3);
    }*/

    // Display subsystem values
    SmartDashboard.putNumber("Coral/Arm/Target Position", armCurrentTarget);
    SmartDashboard.putNumber("Coral/Arm/Actual Position", armEncoder.getPosition());
    SmartDashboard.putNumber("Coral/Elevator/Target Position", elevatorCurrentTarget);
    SmartDashboard.putNumber("Coral/Elevator/Actual Position", elevatorEncoder.getPosition());
    SmartDashboard.putNumber("Coral/Intake/Applied Output", intakeMotor.getAppliedOutput());
  }

  
}