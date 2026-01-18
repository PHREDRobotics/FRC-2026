package frc.robot.subsystems.climb;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimberSubsystem extends SubsystemBase {

  private SparkMax climberMotor;
  private RelativeEncoder climberEncoder;

  // Constructor
  public ClimberSubsystem() {
    climberMotor = new SparkMax(Constants.ClimberConstants.kClimberMotorCANId, MotorType.kBrushless);
    climberEncoder = climberMotor.getEncoder();
  }

  public void resetEncoders() {
    climberEncoder.setPosition(0);
  }

  public double getClimberEncoder() {
    return climberEncoder.getPosition();
  }

  public void startClimberExtend() {
    climberMotor.set(Constants.ClimberConstants.kClimberExtendPower);
  }

  public void startClimberRetract() {
    climberMotor.set( - Constants.ClimberConstants.kClimberRetractPower);

  }

  public void stopClimber() {
    climberMotor.set(0);
  }

  public boolean isClimberExtended() {
    return climberEncoder.getPosition() <= Constants.ClimberConstants.kClimberRaisedEncoderValue;
  }

  public boolean isClimberRetracted() {
    return climberEncoder.getPosition() >= Constants.ClimberConstants.kClimberLoweredEncoderValue;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("climb Power", climberMotor.get());
    SmartDashboard.putNumber("climb Encoder", climberEncoder.getPosition());
  }
}