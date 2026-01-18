package frc.robot.subsystems.Fuel;

import frc.robot.Configs;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class IntakeSubsystem {


  public SparkMax intakeMotorSparkMax = new SparkMax(IntakeConstants.kIntakeMotorCANId, MotorType.kBrushless);
  public SparkMax intakeMovementSparkMax = new SparkMax(IntakeConstants.kIntakeMovementCANId, MotorType.kBrushless);


public IntakeSubsystem() {
    intakeMotorSparkMax.configure(Configs.IntakeMovementSparkMax.motorConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    intakeMovementSparkMax.configure(Configs.IntakeMovementSparkMax.motorConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
  }

  public void startIntake() {
    intakeMotorSparkMax.set(IntakeConstants.kIntakeSpeed);
  }

    public void stopIntake() {
    intakeMotorSparkMax.set(0);
  }

  public void moveIntakeUp() {
    intakeMovementSparkMax.set(IntakeConstants.kIntakeMovementSpeed);
  }

  public void moveIntakeDown() {
    intakeMovementSparkMax.set(-IntakeConstants.kIntakeMovementSpeed);
  }

  public void stopIntakeArmMovement() {
    intakeMovementSparkMax.set(0);
  }

  public boolean isIntakeUp() {
    return false;
  }

  public boolean isIntakeDown() {
    return false;
  }
}
