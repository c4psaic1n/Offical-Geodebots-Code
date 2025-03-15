                                                                                                                                                                  // Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;



import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.DriveSubsystem;



/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */

public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final CoralSubsystem m_coralSubSystem = new CoralSubsystem();
  private final AlgaeSubsystem m_algaeSubsystem = new AlgaeSubsystem();
  

  // The driver's controller
  CommandXboxController m_driverController = new CommandXboxController(OIConstants.kDriverControllerPort);
  //Operator controller
  XboxController m_operatorController = new XboxController(OIConstants.kOperatorControllerPort); 


  /* Button values are;
 * A = 1
 * B = 2
 * X = 3
 * Y = 4
 * LEFT BUMPER = 5
 * RIGHT BUMPER = 6
 */
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    //button configurations CAN change i just dont know what to put for it sooo



//CHANGE TO CHIEF DELPHI SUGGESTION !!!!!!!!!!!!!!
    //algae controls
    new JoystickButton(m_driverController.getHID(), 1) // "A" button
    .whileTrue(m_algaeSubsystem.runIntakeCommand()); // Runs the intake forward

    new JoystickButton(m_driverController.getHID(), 2) // "B" button
    .whileTrue(m_algaeSubsystem.reverseIntakeCommand()); // Runs the intake in reverse




    //coral/elevator controls
    new JoystickButton(m_operatorController, 3) // "X" button
    .whileTrue(m_coralSubSystem.runIntakeCommand()); // Runs the intake forward
    
    new JoystickButton(m_operatorController, 4) // "Y" button
    .whileTrue(m_coralSubSystem.reverseIntakeCommand()); // Runs the intake in reverse

    new JoystickButton(m_operatorController, 1) // "A" button
    .onTrue(m_coralSubSystem.armSetpointLevel2()); // moves coral arm for lvl 2

    new JoystickButton(m_operatorController, 2) // "B" button
    .onTrue(m_coralSubSystem.armSetpointFeeder()); // moves coral arm for lvl 3
    
    // Elevator controls
    // Bind D-pad up to increment the setpoint
    new POVButton(m_operatorController, 0) // D-pad up
    .whileTrue(m_coralSubSystem.setElevatorSpeed(0.8));

    // Bind D-pad down to decrement the setpoint
    new POVButton(m_operatorController, 180) // D-pad down
    .whileTrue(m_coralSubSystem.setElevatorSpeed(-0.5));

    new POVButton(m_operatorController, 0) // D-pad up
    .onFalse(m_coralSubSystem.stopElevator());

    // Bind D-pad down to decrement the setpoint
    new POVButton(m_operatorController, 180) // D-pad down
    .onFalse(m_coralSubSystem.stopElevator());
        
  
    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
                true),
            m_robotDrive));

           //Set the ball intake to in/out when not running based on internal state
            m_algaeSubsystem.setDefaultCommand(m_algaeSubsystem.idleCommand());
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {

    //m_driverController.y().onTrue(getAutonomousCommand());
            
            
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Create config for trajectory
    /*TrajectoryConfig config = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DriveConstants.kDriveKinematics);

    // An example trajectory to follow. All units in meters.
    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(3, 0, new Rotation2d(0)),
        config);

    var thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
        exampleTrajectory,
        m_robotDrive::getPose, // Functional interface to feed supplier
        DriveConstants.kDriveKinematics,

        // Position controllers
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController,
        m_robotDrive::setModuleStates,
        m_robotDrive);

    // Reset odometry to the starting pose of the trajectory.
    m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false));*/
    return new RunCommand(() -> m_robotDrive.drive(
      -0.5,
      0,
      0,
      false),
  m_robotDrive).withTimeout(3);
  }
}
