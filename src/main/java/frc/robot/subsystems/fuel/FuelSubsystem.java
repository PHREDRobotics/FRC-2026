package frc.robot.subsystems.fuel;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants;

public class FuelSubsystem extends SubsystemBase {
  private SparkMax m_intakeMotor = new SparkMax(Constants.FuelConstants.kIntakeMotorCANId, MotorType.kBrushless);
  private SparkMax m_hopperMotor = new SparkMax(Constants.FuelConstants.kHopperMotorCANId, MotorType.kBrushless);
  private SparkMax m_vectorMotor = new SparkMax(Constants.FuelConstants.kVectorMotorCANId, MotorType.kBrushless);
  private SparkMax m_feederLeftMotor = new SparkMax(Constants.FuelConstants.kFeederLeftMotorCANId, MotorType.kBrushless);
  private SparkMax m_feederRightMotor = new SparkMax(Constants.FuelConstants.kFeederRightMotorCANId,MotorType.kBrushless);

  public FuelSubsystem() {
    m_feederLeftMotor.configure(Configs.FeederConfig.feederMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    m_feederRightMotor.configure(Configs.FeederConfig.feederMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  public void intake() {
    m_intakeMotor.set(Constants.FuelConstants.kIntakeSpeedSetting);
    m_hopperMotor.set(Constants.FuelConstants.kHopperSpeedSetting);
    m_vectorMotor.set(Constants.FuelConstants.kVectorSpeedSetting);
  }

  public void feed() {
    m_hopperMotor.set(Constants.FuelConstants.kHopperSpeedSetting);
    m_vectorMotor.set(Constants.FuelConstants.kVectorSpeedSetting);
    m_feederLeftMotor.set(Constants.FuelConstants.kFeederSpeed);
    m_feederRightMotor.set(Constants.FuelConstants.kFeederSpeed);
  }

  public void startOuttake() {
    m_intakeMotor.set(-Constants.FuelConstants.kIntakeSpeedSetting);
  }

  @Override
  public void periodic() {

  }
}