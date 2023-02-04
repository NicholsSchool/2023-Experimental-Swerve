package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.SwerveDrive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import java.util.List;

/* All subsystems, commands, and button mappings should be declared within this class. */
public class RobotContainer {

  // Declare Subsystems:
  private final SwerveDrive m_swerveDrive = new SwerveDrive();

  // Xbox Controllers:
  XboxController m_driverXBoxController = new XboxController(OIConstants.kDriverUSBPort);
  XboxController m_operatorXBoxController = new XboxController(OIConstants.kOperatorUSBPort);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    // Configure all button bindings.
    configureButtonBindings();

    // Configure default commands. (Note: Be sure to set the default drive command.)
    m_swerveDrive.setDefaultCommand(
        new RunCommand(
            () -> m_swerveDrive.drive(
                -MathUtil.applyDeadband(m_driverXBoxController.getLeftY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverXBoxController.getLeftX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverXBoxController.getRightX(), OIConstants.kDriveDeadband),
                true, true),
            m_swerveDrive));
  }

  /**
   * Define all button->command() mappings.
   * 
   * Create a {@link edu.wpi.first.wpilibj.GenericHID} or
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}),
   * then pass the button to a {@link JoystickButton} object.
   */
  private void configureButtonBindings() {

    // Driver XBOX Right Bumper: Set swerve drive to go into defensive X-mode.
    new JoystickButton(m_driverXBoxController, Button.kRightBumper.value)
        .whileTrue(new RunCommand(() -> m_swerveDrive.setX(), m_swerveDrive));

  }

  /**
   * Pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {

    // Create config for trajectory
    TrajectoryConfig config = new TrajectoryConfig(
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
        m_swerveDrive::getPose, // Functional interface to feed supplier
        DriveConstants.kDriveKinematics,

        // Position controllers
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController,
        m_swerveDrive::setModuleStates,
        m_swerveDrive);

    // Reset odometry to the starting pose of the trajectory.
    m_swerveDrive.resetOdometry(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return swerveControllerCommand.andThen(() -> m_swerveDrive.drive(0, 0, 0, false, false));
  }
}
