package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import frc.robot.Constants.ModuleConstants;

public class SwerveModule {

  private final CANSparkMax m_drivingMotor;
  private final CANSparkMax m_turningMotor;

  private final RelativeEncoder m_drivingEncoder;
  private final AbsoluteEncoder m_turningEncoder;

  private double m_angularOffset;

  private final SparkMaxPIDController m_drivingPID;
  private final SparkMaxPIDController m_turningPID;

  // Set the initial module state to 0.0 speed at 0 degrees.
  private SwerveModuleState m_desiredState = new SwerveModuleState(
      0.0,
      new Rotation2d());

  /**
   * Constructs a REV MAXSwerve Module and sets the motors, encoders, and PIDs.
   * 
   * @param drivingCANId
   * @param turningCANId
   * @param angularOffset
   */
  public SwerveModule(int drivingCANId, int turningCANId, double angularOffset) {

    // Create a pair of SPARK MAX motor controllers (driving, turning).
    m_drivingMotor = new CANSparkMax(drivingCANId, MotorType.kBrushless);
    m_drivingMotor.restoreFactoryDefaults();
    m_turningMotor = new CANSparkMax(turningCANId, MotorType.kBrushless);
    m_turningMotor.restoreFactoryDefaults();

    // Setup encoder and PID for the driving motor.
    m_drivingEncoder = m_drivingMotor.getEncoder();
    m_drivingPID = m_drivingMotor.getPIDController();
    m_drivingPID.setFeedbackDevice(m_drivingEncoder);

    // Setup absolute encoder and PID for the turning motor.
    m_turningEncoder = m_turningMotor.getAbsoluteEncoder(Type.kDutyCycle);
    m_turningPID = m_turningMotor.getPIDController();
    m_turningPID.setFeedbackDevice(m_turningEncoder);

    // Apply position and velocity conversion factors for the driving encoder.
    m_drivingEncoder.setPositionConversionFactor(ModuleConstants.kDrivingEncoderPositionFactor);
    m_drivingEncoder.setVelocityConversionFactor(ModuleConstants.kDrivingEncoderVelocityFactor);

    // Apply position and velocity conversion factors for the turning encoder.
    m_turningEncoder.setPositionConversionFactor(ModuleConstants.kTurningEncoderPositionFactor);
    m_turningEncoder.setVelocityConversionFactor(ModuleConstants.kTurningEncoderVelocityFactor);

    // Invert the turning encoder.
    m_turningEncoder.setInverted(ModuleConstants.kTurningEncoderInverted);

    // Optimization: Allow PID controller to go through 0 to get to setpoint.
    m_turningPID.setPositionPIDWrappingEnabled(true);
    m_turningPID.setPositionPIDWrappingMinInput(ModuleConstants.kTurningEncoderPositionPIDMinInput);
    m_turningPID.setPositionPIDWrappingMaxInput(ModuleConstants.kTurningEncoderPositionPIDMaxInput);

    // Set the PID gains for the DRIVING motor.
    m_drivingPID.setP(ModuleConstants.kDrivingP);
    m_drivingPID.setI(ModuleConstants.kDrivingI);
    m_drivingPID.setD(ModuleConstants.kDrivingD);
    m_drivingPID.setFF(ModuleConstants.kDrivingFF);
    m_drivingPID.setOutputRange(ModuleConstants.kDrivingMinOutput, ModuleConstants.kDrivingMaxOutput);

    // Set the PID gains for the TURNING motor.
    m_turningPID.setP(ModuleConstants.kTurningP);
    m_turningPID.setI(ModuleConstants.kTurningI);
    m_turningPID.setD(ModuleConstants.kTurningD);
    m_turningPID.setFF(ModuleConstants.kTurningFF);
    m_turningPID.setOutputRange(ModuleConstants.kTurningMinOutput, ModuleConstants.kTurningMaxOutput);

    // Set idle mode settings for the motor controller.
    m_drivingMotor.setIdleMode(IdleMode.kBrake);
    m_turningMotor.setIdleMode(IdleMode.kBrake);

    // Make Darren happy by limiting the current for the swerve drive modules.
    m_drivingMotor.setSmartCurrentLimit(ModuleConstants.kDrivingMotorCurrentLimit);
    m_turningMotor.setSmartCurrentLimit(ModuleConstants.kTurningMotorCurrentLimit);

    // Save the SPARK MAX configurations in case of temp browns out.
    m_drivingMotor.burnFlash();
    m_turningMotor.burnFlash();

    // Set the angular offset for the module and the desired angle.
    m_angularOffset = angularOffset;
    m_desiredState.angle = new Rotation2d(m_turningEncoder.getPosition());

    // Set driving rotation counter to zero
    m_drivingEncoder.setPosition(0);
  }

  /**
   * Returns the current state of the module.
   *
   * @return SwerveModuleState (driving velocity RPMs, turning angle)
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(
        m_drivingEncoder.getVelocity(),
        new Rotation2d(m_turningEncoder.getPosition() - m_angularOffset));
  }

  /**
   * Returns the current position of the module.
   *
   * @return SwerveModulePosition (driving motor rotations, turning angle)
   */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        m_drivingEncoder.getPosition(),
        new Rotation2d(m_turningEncoder.getPosition() - m_angularOffset));
  }

  /**
   * Sets the desired state for the module (speed in m/s, angle in radians).
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {

    // Apply chassis angular offset to the desired state..
    SwerveModuleState updatedDesiredState = new SwerveModuleState();
    updatedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
    updatedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(m_angularOffset));

    // Optimization to prevent the turning motor spinning further than 90 degrees.
    SwerveModuleState optimizedDesiredState = SwerveModuleState.optimize(
        updatedDesiredState,
        new Rotation2d(m_turningEncoder.getPosition()));

    // Set the driving and turning motor PIDs to the optimized setpoints.
    m_drivingPID.setReference(optimizedDesiredState.speedMetersPerSecond, CANSparkMax.ControlType.kVelocity);
    m_turningPID.setReference(optimizedDesiredState.angle.getRadians(), CANSparkMax.ControlType.kPosition);

    // Note: Not sure if this is correct, may need to set as optimizedDesiredState.
    m_desiredState = desiredState;
  }

  /** Zeroes all the SwerveModule encoders. */
  public void resetEncoders() {
    m_drivingEncoder.setPosition(0);
  }

}
