package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.AlgaeSubsystemConstants;





public class AlgaeSubsystem extends SubsystemBase {

  // Initialize arm SPARK. We will use MAXMotion position control for the arm, so we also need to
  // initialize the closed loop controller and encoder.
  private SparkFlex armMotor = new SparkFlex(AlgaeSubsystemConstants.kPivotMotorCanId, MotorType.kBrushless);
  private SparkClosedLoopController armController = armMotor.getClosedLoopController();
  private RelativeEncoder armEncoder = armMotor.getEncoder(); 
  
  // Initialize intake SPARK. We will use open loop control for this so we don't need a closed loop
  // controller like above.
  private SparkFlex intakeMotor =new SparkFlex(AlgaeSubsystemConstants.kIntakeMotorCanId, MotorType.kBrushless);

  // Member variables for subsystem state management
  private boolean stowWhenIdle = true;
  private boolean wasReset = false;
  private boolean isRunningIntake = false;
  private boolean isReversingIntake = false;







 private final Timer timer = new Timer();




  public AlgaeSubsystem() {
    




 


     /* Apply the configuration to the SPARKs.
     *
     * kResetSafeParameters is used to get the SPARK to a known state. This
     * is useful in case the SPARK is replaced.
     *
     * kPersistParameters is used to ensure the configuration is not lost when
     * the SPARK loses power. This is useful for power cycles that may occur
     * mid-operation.
     */
    intakeMotor.configure(
        Configs.AlgaeSubsystem.intakeConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    armMotor.configure(
        Configs.AlgaeSubsystem.armConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);





   





    // Zero arm encoder on initialization, so that means where the arm 
    //is when the code is enabled is where its "0" is.
    armEncoder.setPosition(2);







    
  }




  // Zero the arm encoder when the user button is pressed on the roboRIO *
  private void zeroOnUserButton() {
    if (!wasReset && RobotController.getUserButton()) {
      // Zero the encoder only when button switches from "unpressed" to "pressed" to prevent
      // constant zeroing while pressed
      wasReset = true;
      armEncoder.setPosition(0);
    } else if (!RobotController.getUserButton()) {
      wasReset = false;
    }
  }


  /** Set the intake motor power in the range of [-1, 1]. */
  private void setIntakePower(double power) {
    intakeMotor.set(power); //hopefully it uses the "power/constant" that is set for that line of
    //code using the setIntakePower command
  }
  
  /** Set the arm motor position. This will use closed loop position control. */
  private void setIntakePosition(double position) {
    armController.setReference(position, ControlType.kPosition);
  }



 
  
    
 

  public void updateIntakeState() {
    if (isRunningIntake) {  //isrunning(action) is a boolean that is from a class [rovided by the vendordeps
      setIntakePower(AlgaeSubsystemConstants.IntakeSetpoints1.kForward);
      setIntakePosition(AlgaeSubsystemConstants.ArmSetpoints1.kDown);
    } else if (isReversingIntake) {
      setIntakePower(AlgaeSubsystemConstants.IntakeSetpoints1.kReverse);//
      setIntakePosition(AlgaeSubsystemConstants.ArmSetpoints1.kHold);
    }  else if (stowWhenIdle) {
      setIntakePower (AlgaeSubsystemConstants.IntakeSetpoints1.kForward );
      setIntakePosition(AlgaeSubsystemConstants.ArmSetpoints1.kStow);
  }
}
   





/**
 * Command to run the algae intake. This will extend the arm to its "down" position and run the
 * motor at its "forward" power to intake the ball.
 *
 * <p>This will also update the idle state to hold onto the ball when this command is not running.
 */


  public Command runIntakeCommand() {
    return this.runOnce(
        () -> {
          stowWhenIdle = false;
          isRunningIntake = true;
          isReversingIntake = false;
          updateIntakeState();
        });
  }

  /**
   * Command to run the algae intake in reverse. This will extend the arm to its "hold" position and
   * run the motor at its "reverse" power to eject the ball.
   *
   * <p>This will also update the idle state to stow the arm when this command is not running.
   */
  public Command reverseIntakeCommand() {
    return this.runOnce(
        () -> {
          stowWhenIdle = false;
          isRunningIntake = false;
          isReversingIntake = true;
          updateIntakeState();
        });
  }



  
  /**
   * Command to run when the intake is not actively running. When in the "hold" state, the intake
   * will stay in the "hold" position and run the motor at its "hold" power to hold onto the ball.
   * When in the "stow" state, the intake will stow the arm in the "stow" position and stop the
   * motor.
   */
  public Command idleCommand() {
    return this.run(
        () -> {
          updateIntakeState();
        });
  }



  @Override
  public void periodic() {
    zeroOnUserButton();
    // Display subsystem values
    SmartDashboard.putNumber("Algae/Arm/Position", armEncoder.getPosition());
    SmartDashboard.putNumber("Algae/Intake/Applied Output", intakeMotor.getAppliedOutput());

    
  }

 
}