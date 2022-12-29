package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

import java.util.function.DoubleSupplier;

public class SwerveDrive extends CommandBase {
    private final Drivetrain drivetrain;

    private final DoubleSupplier m_translationXSupplier;
    private final DoubleSupplier m_translationYSupplier;
    private final DoubleSupplier m_rotationSupplier;

    public SwerveDrive(Drivetrain drivetrainSubsystem,
            DoubleSupplier translationXSupplier,
            DoubleSupplier translationYSupplier,
            DoubleSupplier rotationSupplier) {
        this.drivetrain = drivetrainSubsystem;
        this.m_translationXSupplier = translationXSupplier;
        this.m_translationYSupplier = translationYSupplier;
        this.m_rotationSupplier = rotationSupplier;

        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void execute() {
        drivetrain.drive(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                        drivetrain.percentOutputToMetersPerSecond(m_translationXSupplier.getAsDouble()),
                        drivetrain.percentOutputToMetersPerSecond(m_translationYSupplier.getAsDouble()),
                        drivetrain.percentOutputToRadiansPerSecond(m_rotationSupplier.getAsDouble()),
                        drivetrain.getYaw2d()));
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
    }
}