package frc.robot.subsystems.climber;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimberSubsystem extends SubsystemBase {

  private SparkMax m_climberMotor;
  private RelativeEncoder m_climberEncoder;
  private double m_encoderValue;

  public void ClimberSubsystem() {
    m_climberMotor = new SparkMax(Constants.ClimberConstants.kClimberMotorCANId, MotorType.kBrushless);
    m_climberEncoder = m_climberMotor.getEncoder();
    m_encoderValue = m_climberEncoder.getPosition();
  }

  public void resetEncoders() {
    m_climberEncoder.setPosition(0);
  }

  public double getClimberEncoder() {
    return m_encoderValue;
  }

  public void startClimberExtend() {
    m_climberMotor.set(Constants.ClimberConstants.kClimberExtendPower);
  }

  public void startClimberRetract() {
    m_climberMotor.set(-Constants.ClimberConstants.kClimberRetractPower);

  }

  public void stopClimber() {
    m_climberMotor.set(0);
  }

  public boolean isClimberExtended() {
    return m_encoderValue <= Constants.ClimberConstants.kClimberRaisedEncoderValue;
  }

  public boolean isRobotClimbed() {
    return m_encoderValue <= Constants.ClimberConstants.kClimberClimbedEncoderValue;
  }

  public boolean isClimberRetracted() {
    return m_encoderValue >= Constants.ClimberConstants.kClimberLoweredEncoderValue;
  }

  @Override
  public void periodic() {
    m_encoderValue = m_climberEncoder.getPosition();
    SmartDashboard.putNumber("climb Power", m_climberMotor.get());
    SmartDashboard.putNumber("climb Encoder", m_encoderValue);
  }
}