package frc.robot.commands;
/*
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;

public class AutoShootCommand extends Command {
    private ShooterSubsystem m_shooterSubsystem;
    private SwerveSubsystem m_swerveSubsystem;
    private VisionSubsystem m_visionSubsystem;
    private DoubleSupplier x;
    private DoubleSupplier y;
    private boolean shoot;

    public AutoShootCommand(ShooterSubsystem shooterSubsystem, SwerveSubsystem swerveSubsystem,
            VisionSubsystem visionSubsystem, DoubleSupplier x, DoubleSupplier y, BooleanSupplier shoot) {
        m_shooterSubsystem = shooterSubsystem;
        m_swerveSubsystem = swerveSubsystem;
        m_visionSubsystem = visionSubsystem;
        this.x = x;
        this.y = y;
        this.shoot = shoot.getAsBoolean();

        addRequirements(shooterSubsystem, swerveSubsystem, visionSubsystem);
    }

    @Override
    public void initialize() {
        new AlignTagCommand(m_swerveSubsystem, m_visionSubsystem, x, y);
    }

    @Override
    public void execute() {
        if (shoot) {
            new ShootCommand(m_shooterSubsystem, );
        }
    }
}
*/