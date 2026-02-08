package frc.robot.subsystems.shooter;

import frc.robot.Configs;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class ShooterSubsystem extends SubsystemBase {
  //private SparkMax m_feederLeftSparkMax = new SparkMax(ShooterConstants.kFeederLeftMotorCANId, MotorType.kBrushless);
  //private SparkMax m_feederRightSparkMax = new SparkMax(ShooterConstants.kFeederRightMotorCANId, MotorType.kBrushless);

  private SparkMax m_shooterLeftSparkMax = new SparkMax(ShooterConstants.kShooterLeftMotorCANId, MotorType.kBrushless);
  private SparkMax m_shooterRightSparkMax = new SparkMax(ShooterConstants.kShooterRightMotorCANId, MotorType.kBrushless);

  private SparkClosedLoopController m_shooterLeftPID;
  private SparkClosedLoopController m_shooterRightPID;

  private double shootSpeed;

  // public SparkMax frontLeftMotorSparkMax = new
  // SparkMax(ShooterConstants.kShooterFrontLeftMotorCANId, MotorType.kBrushless);
  // public SparkMax shooterFrontRightMotorSparkMax = new
  // SparkMax(ShooterConstants.kShooterFrontRightMotorCANId,MotorType.kBrushless);

  public ShooterSubsystem() {
    //m_feederLeftSparkMax.configure(Configs.FeederConfig.feederMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    //m_feederRightSparkMax.configure(Configs.FeederConfig.feederMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

    m_shooterLeftSparkMax.configure(Configs.ShooterConfig.shooterMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    m_shooterRightSparkMax.configure(Configs.ShooterConfig.shooterMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

    m_shooterLeftPID = m_shooterLeftSparkMax.getClosedLoopController();
    m_shooterRightPID = m_shooterRightSparkMax.getClosedLoopController();
  }

  public void feed() {
    //m_feederLeftSparkMax.set(ShooterConstants.kFeederSpeed);
    //m_feederRightSparkMax.set(ShooterConstants.kFeederSpeed);
  }

  public void shoot(double shootSpeedRPM) {
    this.shootSpeed = shootSpeedRPM;

    m_shooterLeftPID.setSetpoint(-this.shootSpeed, ControlType.kVelocity);
    m_shooterRightPID.setSetpoint(this.shootSpeed, ControlType.kVelocity);
  }

  public void stop() {
    //m_feederLeftSparkMax.stopMotor();
    //m_feederRightSparkMax.stopMotor();
    m_shooterLeftSparkMax.stopMotor();
    m_shooterRightSparkMax.stopMotor();
  }

  public boolean isAtSpeed() {
    return (m_shooterLeftSparkMax.getEncoder().getVelocity() > shootSpeed - Constants.ShooterConstants.kShootThreshold) && (m_shooterLeftSparkMax.getEncoder().getVelocity() < shootSpeed + Constants.ShooterConstants.kShootThreshold); 
  }

  /**
   * Gets the power for the shooter to shoot at the hub based on a distance in meters
   * @return
   */
  public double getShootPower(double distance) {
    return Constants.ShooterConstants.kAutoShooterDistanceMultiplier
        * Math.pow(distance, Constants.ShooterConstants.kAutoShooterDistanceExponent);
  }

  @Override
  public void periodic() {
    super.periodic();

    SmartDashboard.putNumber("sparkmax encoder", m_shooterLeftSparkMax.getEncoder().getPosition());
    SmartDashboard.putNumber("velocity", m_shooterLeftSparkMax.getEncoder().getVelocity());

  }
}
