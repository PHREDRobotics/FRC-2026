// package frc.robot.subsystems.fuel;

// import com.revrobotics.spark.SparkLowLevel.MotorType;
// import com.revrobotics.spark.SparkMax;

// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants;

// public class FuelSubsystem extends SubsystemBase {

//   private SparkMax m_intakeMotor;
//   private SparkMax m_hopperMotor;
//   private SparkMax m_vectorMotor;

//   public FuelSubsystem() {
//     m_intakeMotor = new SparkMax(Constants.FuelConstants.kIntakeMotorCANId, MotorType.kBrushless);
//     m_hopperMotor = new SparkMax(Constants.FuelConstants.kHopperMotorCANId, MotorType.kBrushless);
//     m_vectorMotor = new SparkMax(Constants.FuelConstants.kVectorMotorCANId, MotorType.kBrushless);
//   }

//   public void startIntake() {
//     m_intakeMotor.set(Constants.FuelConstants.kIntakeSpeedSetting);
//     m_hopperMotor.set(Constants.FuelConstants.kHopperSpeedSetting);
//     m_vectorMotor.set(Constants.FuelConstants.kVectorSpeedSetting);
//   }

//   public void stopIntake() {
//     m_intakeMotor.set(0);
//     m_hopperMotor.set(0);
//     m_vectorMotor.set(0);

//   }

//   public void startOuttake() {
//     m_intakeMotor.set(- Constants.FuelConstants.kIntakeSpeedSetting);
//   }

//   @Override
//   public void periodic() {

//   }
// }