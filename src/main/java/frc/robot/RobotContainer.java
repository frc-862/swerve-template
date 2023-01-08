package frc.robot;

import edu.wpi.first.wpilibj.XboxController;

import java.util.HashMap;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lightningUtil.LightningContainer;
import frc.lightningUtil.auto.AutonomousCommandFactory;
import frc.lightningUtil.filter.JoystickFilter;
import frc.lightningUtil.filter.JoystickFilter.Mode;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.XboxControllerConstants;
import frc.robot.commands.SwerveDrive;
import frc.robot.subsystems.Drivetrain;

public class RobotContainer extends LightningContainer {
    // creates our drivetrain subsystem
    private static final Drivetrain drivetrain = new Drivetrain();

    // creates our driver controller and deadzone
    private static final XboxController driver = new XboxController(0);
    private static final JoystickFilter joystickFilter = new JoystickFilter(XboxControllerConstants.DEADBAND,
            XboxControllerConstants.MIN_POWER, XboxControllerConstants.MAX_POWER, Mode.CUBED);

    private HashMap<String, Command> eventMap = new HashMap<>();

    // configure the button bindings
    @Override
    protected void configureButtonBindings() {
        // back button to reset feild centeric driving to current heading of the robot
        new Trigger(driver::getBackButton)
                .onTrue(new InstantCommand(drivetrain::zeroYaw, drivetrain));
    }

    // creates the autonomous commands
    @Override
    protected void configureAutonomousCommands() {
        // creates a trajectory using pathplanner
        AutonomousCommandFactory.makeTrajectory("meter", DrivetrainConstants.MAX_VELOCITY_METERS_PER_SECOND,
                DrivetrainConstants.MAX_VELOCITY_METERS_PER_SECOND, drivetrain);
    }

    @Override
    protected void configureDefaultCommands() {
        // set up the default command for the drivetrain.
        // the controls are for field-oriented driving:
        // left stick Y axis -> forward and backwards movement
        // left stick X axis -> left and right movement
        // right stick X axis -> rotation
        drivetrain.setDefaultCommand(
                new SwerveDrive(drivetrain, () -> -joystickFilter.filter(driver.getLeftX()),
                        () -> joystickFilter.filter(driver.getLeftY()),
                        () -> -joystickFilter.filter(driver.getRightX())));

    }

    @Override
    protected void configureSystemTests() {
        // configure the button bindings
    }

    @Override
    protected void releaseDefaultCommands() {
        // TODO Auto-generated method stub

    }

    @Override
    protected void initializeDashboardCommands() {
        // TODO Auto-generated method stub

    }

    @Override
    protected void configureFaultCodes() {
        // TODO Auto-generated method stub

    }

    @Override
    protected void configureFaultMonitors() {
        // TODO Auto-generated method stub

    }
}