package frc.robot;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.LimitSwitchConfig.Type;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.util.Units;

import com.revrobotics.spark.config.SparkFlexConfig;
import frc.robot.Constants.ModuleConstants;

public final class Configs {
    public static final class MAXSwerveModule {
        public static final SparkFlexConfig drivingConfig = new SparkFlexConfig();
        public static final SparkMaxConfig turningConfig = new SparkMaxConfig();

        static {
            // Use module constants to calculate conversion factors and feed forward gain.
            double drivingFactor = ModuleConstants.kWheelDiameterMeters * Math.PI
                    / ModuleConstants.kDrivingMotorReduction;
            double turningFactor = 2 * Math.PI;
            double drivingVelocityFeedForward = 1 / ModuleConstants.kDriveWheelFreeSpeedRps;

            drivingConfig
                    .idleMode(IdleMode.kBrake)
                    .smartCurrentLimit(80);
            drivingConfig.encoder
                    .positionConversionFactor(drivingFactor) // meters
                    .velocityConversionFactor(drivingFactor / 60.0); // meters per second
            drivingConfig.closedLoop
                    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                    // These are example gains you may need to them for your own robot!
                    .pid(0.04, 0, 0)
                    .velocityFF(drivingVelocityFeedForward)
                    .outputRange(-1, 1);

            turningConfig
                    .idleMode(IdleMode.kBrake)
                    .smartCurrentLimit(20);
            turningConfig.absoluteEncoder
                    // Invert the turning encoder, since the output shaft rotates in the opposite
                    // direction of the steering motor in the MAXSwerve Module.
                    .inverted(true)
                    .positionConversionFactor(turningFactor) // radians
                    .velocityConversionFactor(turningFactor / 60.0); // radians per second
            turningConfig.closedLoop
                    .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                    // These are example gains you may need to them for your own robot!
                    .pid(1, 0, 0)
                    .outputRange(-1, 1)
                    // Enable PID wrap around for the turning motor. This will allow the PID
                    // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
                    // to 10 degrees will go through 0 rather than the other direction which is a
                    // longer route.
                    .positionWrappingEnabled(true)
                    .positionWrappingInputRange(0, turningFactor);
        }
    }

    public static final class Arm {
        public static final SparkMaxConfig armConfig = new SparkMaxConfig();

        static {

                double armFactor = 2*Math.PI;
                // Configure basic settings of the arm motor
                armConfig
                  .idleMode(IdleMode.kCoast)
                  .inverted(true)
                  .smartCurrentLimit(20)
                  .voltageCompensation(10);
        
                armConfig.absoluteEncoder
                  .setSparkMaxDataPortConfig()
                  .positionConversionFactor(armFactor)
                  .velocityConversionFactor(armFactor/60)
                  .inverted(true);
                /*
                 * Configure the closed loop controller. We want to make sure we set the
                 * feedback sensor as the primary encoder.
                 */
                armConfig
                    .closedLoop
                    .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                    .positionWrappingEnabled(true)
                    .positionWrappingInputRange(0, armFactor)
                    // Set PID values for position control
                    .p(1.2)
                    .d(0.075)
                    .outputRange(-1, 1)
                    .maxMotion
                    // Set MAXMotion parameters for position control
                    .maxVelocity(300)
                    .maxAcceleration(900)
                    .allowedClosedLoopError(0.1);

                armConfig
                    .signals
                    .absoluteEncoderPositionAlwaysOn(true)
                    .faultsAlwaysOn(true)
                    .warningsAlwaysOn(true);
        }
    }
    
    public static final class Elevator {
        public static final SparkMaxConfig elevatorConfig = new SparkMaxConfig();

        static{
                double gearRatio = 12.0*14.0/16.0;
                double sprocketDistanceMeters = Units.inchesToMeters(6.5);
                double countsPerRevolution = 8192;
                
                elevatorConfig
                  .idleMode(IdleMode.kCoast)
                  .smartCurrentLimit(50)
                  .inverted(true)
                  .voltageCompensation(10);
                elevatorConfig.encoder
                  .positionConversionFactor(sprocketDistanceMeters/gearRatio)
                  .velocityConversionFactor(sprocketDistanceMeters/gearRatio/60);
                elevatorConfig.alternateEncoder
                  .countsPerRevolution(8192)
                  .positionConversionFactor(sprocketDistanceMeters)
                  .velocityConversionFactor(sprocketDistanceMeters/60);
                elevatorConfig.closedLoop
                  .feedbackSensor(FeedbackSensor.kAlternateOrExternalEncoder)
                  .p(3)
                  .outputRange(-1, 1);
                elevatorConfig.closedLoop.maxMotion
                  .maxVelocity(50) // m/s Not sure what exact units are in??
                  .maxAcceleration(300)
                  .allowedClosedLoopError(0.004);  // 4mm
        }
}

public static final class Intake {
        public static final SparkMaxConfig intakeConfig = new SparkMaxConfig();

        static {
                // Configure basic settings of the arm motor
                intakeConfig.idleMode(IdleMode.kCoast).smartCurrentLimit(20).voltageCompensation(10);
        }}

public static final class Algae {
        public static final SparkMaxConfig algaeConfig = new SparkMaxConfig();
        
        static {
                // Configure basic settings of the algae motor
                algaeConfig
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(60)
                .voltageCompensation(12);
        }}
public static final class AlgaeIntake {
        public static final SparkMaxConfig algaeIntakeConfig = new SparkMaxConfig();
        
        static {
                // Configure basic settings of the algaeIntake motor
                algaeIntakeConfig
                .idleMode(IdleMode.kCoast)
                .smartCurrentLimit(60)
                .voltageCompensation(12);
        }}
}