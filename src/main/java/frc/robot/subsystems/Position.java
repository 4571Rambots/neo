package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;

public class Position extends SubsystemBase {
    // Motor instantiated on port 10 (adjust if needed)
    private final SparkMax motor = new SparkMax(10, MotorType.kBrushless);

    // Auto-zero variables
    private boolean autoZeroMode = false;
    private double targetPosition = 0.0;
    private double homePosition = 0.0;
    private long timeWithinTolerance = 0;
    private static final double POSITION_TOLERANCE = 0.01;
    private static final long HOLD_TIME_MS = 150;

    /** Constructor – configure motor and encoder settings **/
    public Position() {
        SparkMaxConfig config = new SparkMaxConfig();
        config.inverted(false)
              .idleMode(IdleMode.kBrake);
        config.encoder
              .positionConversionFactor(1.0) // 1 rotation = 1 unit
              .velocityConversionFactor(1.0);
        config.closedLoop
              .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
              .pid(0.1, 0.0, 0.0); // Tune PID values as needed
        config.closedLoop.maxMotion
              .maxVelocity(1000) // RPM
              .maxAcceleration(500); // RPM/sec

        // Apply configuration and set safe parameters
        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Initialize encoder position and set homePosition
        motor.getEncoder().setPosition(0.0);
        homePosition = motor.getEncoder().getPosition();
    }

    /** Set the current position as home **/
    public void setHome() {
        homePosition = motor.getEncoder().getPosition();
        System.out.println("Home position set to: " + homePosition);
    }

    /** Drive the motor manually (pass a speed value, e.g., from a joystick) **/
    public void manualControl(double speed) {
        // Invert speed if necessary based on motor wiring
        motor.set(-speed);
    }

    /** Activate auto-zero mode to go to home position **/
    public void startAutoZero() {
        autoZeroMode = true;
        timeWithinTolerance = 0;
        targetPosition = homePosition;
        System.out.println("Auto-zero mode activated. Target set to home: " + targetPosition);
    }

    /** Activate mode to go to position 0 **/
    public void goToZero() {
        autoZeroMode = true;
        timeWithinTolerance = 0;
        targetPosition = 90;
        System.out.println("Going to position 0.");
    }
    
    /** Cancel auto-zero mode so manual control can resume **/
    public void cancelAutoZero() {
        if (autoZeroMode) {
            autoZeroMode = false;
            System.out.println("Auto-zero mode canceled due to manual control.");
        }
    }

    /** Update auto-zero mode; this should be called periodically **/
    public void updateAutoZero() {
        if (autoZeroMode) {
            motor.getClosedLoopController().setReference(targetPosition, ControlType.kMAXMotionPositionControl);
            double error = Math.abs(motor.getEncoder().getPosition() - targetPosition);
            if (error < POSITION_TOLERANCE) {
                if (timeWithinTolerance == 0) {
                    timeWithinTolerance = System.currentTimeMillis();
                } else if (System.currentTimeMillis() - timeWithinTolerance >= HOLD_TIME_MS) {
                    autoZeroMode = false;
                    System.out.println("Reached target position: " + targetPosition);
                }
            } else {
                timeWithinTolerance = 0;
            }
        }
    }

    /** Check whether auto-zero mode is active **/
    public boolean isAutoZeroActive() {
        return autoZeroMode;
    }

    @Override
    public void periodic() {
        // Update auto-zero if active
        if (autoZeroMode) {
            updateAutoZero();
        }
        // Print out the current encoder position on every cycle
        System.out.println("Current Encoder Position: " + motor.getEncoder().getPosition());
    }
}
