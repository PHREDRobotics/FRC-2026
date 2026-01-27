package frc.robot.subsystems.shooter;

import frc.robot.Configs;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class ShooterSubsystem extends SubsystemBase {
  // Declare Motor Sparkmaxes

  public SparkMax feederLeftMotorSparkMax = new SparkMax(ShooterConstants.kFeederRightMotorCANId, MotorType.kBrushless);
  public SparkMax feederRightMotorSparkMax = new SparkMax(ShooterConstants.kFeederRightMotorCANId,
      MotorType.kBrushless);

  public SparkMax shooterFrontLeftMotorSparkMax = new SparkMax(ShooterConstants.kShooterFrontLeftMotorCANId,
      MotorType.kBrushless);
  public SparkMax shooterFrontRightMotorSparkMax = new SparkMax(ShooterConstants.kShooterFrontRightMotorCANId,
      MotorType.kBrushless);
  public SparkMax shooterBackLeftMotorSparkMax = new SparkMax(ShooterConstants.kShooterBackLeftMotorCANId,
      MotorType.kBrushless);
  public SparkMax shooterBackRightMotorSparkMax = new SparkMax(ShooterConstants.kShooterBackRightMotorCANId,
      MotorType.kBrushless);

  // Constructor
  public ShooterSubsystem() {

  }

  

  public void feederMotorsStart() {

    feederLeftMotorSparkMax.set(ShooterConstants.kFeederSpeed);
    feederRightMotorSparkMax.set(ShooterConstants.kFeederSpeed);

  }

  public void shootMotorsStart(double shootingSpeed) {
    shooterFrontLeftMotorSparkMax.set(shootingSpeed);
    shooterFrontRightMotorSparkMax.set(shootingSpeed);
    shooterBackLeftMotorSparkMax.set(shootingSpeed);
    shooterBackRightMotorSparkMax.set(shootingSpeed);

  }

  public void stop() {
    feederLeftMotorSparkMax.set(0);
    feederRightMotorSparkMax.set(0);
    shooterFrontLeftMotorSparkMax.set(0);
    shooterFrontRightMotorSparkMax.set(0);
    shooterBackLeftMotorSparkMax.set(0);
    shooterBackRightMotorSparkMax.set(0);

  }

  @Override
  public void periodic() {
      // TODO Auto-generated method stub
      super.periodic();
  }

public void stopShooter() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'stopShooter'");
}

}
