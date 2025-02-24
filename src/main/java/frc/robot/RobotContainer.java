package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Position;

import com.revrobotics.ColorSensorV3;

public class RobotContainer {

    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // max angular velocity

    /* Swerve drive platform setup */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1)
            .withRotationalDeadband(MaxAngularRate * 0.1)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final Telemetry logger = new Telemetry(MaxSpeed);
    private final CommandXboxController joystick = new CommandXboxController(0);
    private final CommandXboxController joysticks = new CommandXboxController(1);
    
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    /* Instantiate Shooter subsystem */
    private final Shooter shooter = new Shooter();
    /* Instantiate Position subsystem */
    private final Position position = new Position();

    /* Path follower */
    private final SendableChooser<Command> autoChooser;

    // --- Color Sensor Setup ---
    private final I2C.Port i2cPort = I2C.Port.kOnboard;
    private final ColorSensorV3 colorSensor = new ColorSensorV3(i2cPort);

    // Flags to track shooter state
    private boolean shooterEnabled = false;

    public RobotContainer() {
        autoChooser = AutoBuilder.buildAutoChooser("Tests");
        SmartDashboard.putData("Auto Mode", autoChooser);
        configureBindings();

        // Set default command for the Position subsystem.
        // This uses the second controller's left Y axis to control position manually
        // In RobotContainer, within your constructor or configuration method:
        position.setDefaultCommand(new RunCommand(() -> {
            double manualSpeed = joysticks.getLeftY();
            // Optionally apply a deadband
            if (Math.abs(manualSpeed) >= 0.1) {
                // Cancel auto-zero mode if any manual input is detected
                position.cancelAutoZero();
                position.manualControl(manualSpeed);
            } else {
                // Optionally, you can hold the motor at zero when there's no input and auto-zero isn't active.
                if (!position.isAutoZeroActive()) {
                    position.manualControl(0.0);
                }
            }
        }, position));


        // Start periodic color sensor check in a dedicated thread
        new Thread(() -> {
            while (true) {
                readColorSensor();
                try {
                    Thread.sleep(50); // Check every 50ms
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }
        }).start();
    }

    private void configureBindings() {
        // Default drivetrain command
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * MaxSpeed)
                     .withVelocityY(-joystick.getLeftX() * MaxSpeed)
                     .withRotationalRate(-joystick.getRightX() * MaxAngularRate)
            )
        );

        joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        joystick.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        ));

        joystick.pov(0).whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(0.5).withVelocityY(0)
        ));
        joystick.pov(180).whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(-0.5).withVelocityY(0)
        ));

        // SysId routines
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // Reset field-centric heading on left bumper press
        joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        // Shooter control on second controller: press A to start shooter
        joysticks.a().whileTrue(new RunCommand(
            () -> {
                if (!shooterEnabled) {
                    shooterEnabled = true;  // Enable the shooter
                    shooter.shoot(1.0);     // Run at full speed
                    System.out.println("Shooter Started");
                }
            },
            shooter
        ));



                // Position control on second controller:
        // Press Y to set the current position as home
        joysticks.y().onTrue(new InstantCommand(() -> position.setHome(), position));
        // Press B to activate "go to zero" (move to position 0)
        joysticks.b().onTrue(new InstantCommand(() -> position.goToZero(), position));


        // Position control on second controller:
        
        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    public void readColorSensor() {
        Color detectedColor = colorSensor.getColor();
        double redValue = detectedColor.red;

        // Define the threshold for white detection
        boolean isWhite = (redValue >= 0.245 && redValue <= 0.252);

        // If white is detected and the shooter is enabled, stop the shooter.
        if (isWhite && shooterEnabled) {
            shooter.shoot(0);         // Stop the motor
            shooterEnabled = false;     // Disable shooter until "A" is pressed again
            System.out.println("White detected - Stopping Shooter");
        }

        // Debug output
        System.out.println("Red Value: " + redValue);
    }
}