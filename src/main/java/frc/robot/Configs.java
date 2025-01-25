package frc.robot;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.LimitSwitchConfig.Type;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import frc.robot.Constants.ArmConstants;
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
                    .smartCurrentLimit(50);
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

    public static final class Brake {
        public static final SparkMaxConfig brakeConfig = new SparkMaxConfig();

        static {
                brakeConfig
                        .smartCurrentLimit(30)
                        .idleMode(IdleMode.kBrake)
                        .inverted(true);
        }
    }

    public static final class Feeder {
        public static final SparkMaxConfig feederConfig = new SparkMaxConfig();

        static {
                feederConfig
                        .smartCurrentLimit(30)
                        .idleMode(IdleMode.kBrake);
                feederConfig.limitSwitch
                        .forwardLimitSwitchType(Type.kNormallyOpen)
                        .forwardLimitSwitchEnabled(false);
        }
    }

    public static final class Launcher {
        public static final SparkFlexConfig launcherConfig = new SparkFlexConfig();

        static {
                launcherConfig
                        .smartCurrentLimit(40)
                        .idleMode(IdleMode.kCoast);
                launcherConfig.closedLoop
                        .pidf(0.00025,0.0,0.00001,0.0001505)
                        .outputRange(-1,1);
        }
    }

    public static final class Underroller {
        public static final SparkMaxConfig underrollerConfig = new SparkMaxConfig();

        static {
                underrollerConfig
                        .smartCurrentLimit(30)
                        .idleMode(IdleMode.kBrake);              
        }
    }

    public static final class Arm {
        public static final SparkMaxConfig armConfig = new SparkMaxConfig();

        static {
                double armPositionFactor = ((2 * Math.PI) / ArmConstants.kArmGearRatio); // radians

                armConfig
                        .smartCurrentLimit(50)
                        .idleMode(IdleMode.kBrake);
                armConfig.encoder
                        .positionConversionFactor(armPositionFactor) // radians
                        .velocityConversionFactor(armPositionFactor / 60.0); // radians/sec
                armConfig.closedLoop
                        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                        .pidf(4,0,0.5,0)
                        .outputRange(-1, 1);


        }
    }

    
}
