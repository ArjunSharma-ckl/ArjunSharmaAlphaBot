package frc.robot.subsystems.coral;

import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Value;

import java.util.function.BooleanSupplier;

import org.lasarobotics.fsm.StateMachine;
import org.lasarobotics.fsm.SystemState;
import org.littletonrobotics.junction.Logger;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;

//ADD STATE MACHINE
public class CoralSubsystem extends StateMachine implements AutoCloseable {

    public static record Hardware (
        SparkMax coralMotor,
        SparkMax armMotor
    ) {}
    //SPEEEEDSSS
    static final Dimensionless INTAKE_MOTOR_SPEED = Percent.of(100);
    static final Dimensionless SCORE_MOTOR_SPEED = Percent.of(-100);
    static final Dimensionless ARM_RETRACT_SPEED = Percent.of(~30);

    public enum CoralSubsystemStates implements SystemState {
        NOTHING {
            @Override
            public SystemState nextState() {
                return this;
            }
        },

        CORAL_INTAKE {
            @Override
            public void initialize() {
                Logger.getInstance().recordOutput("CoralSubsystemState", "CORAL_INTAKE");
            }
            @Override
            public void execute() {
                
            }
            @Override
            public SystemState nextState() {
                return this;
            }
        },

        CORAL_SCORE {
            @Override
            public void initialize() {
                Logger.getInstance().recordOutput("Subsystems/Coral/State", "CORAL_SCORE");
            }

            @Override
            public void execute() {
                
            }
            @Override
            public SystemState nextState() {
                return this;
            }
        },
    }

    private static CoralSubsystem s_coralSubsystemInstance;
    private final SparkMax m_coralMotor;
    private final SparkMaxConfig m_coralMotorConfig;
    private final SparkMax m_armMotor;
    private final SparkClosedLoopController m_armController;
    private final RelativeEncoder m_armEncoder;
    private final SparkMaxConfig m_armMotorConfig;
    private BooleanSupplier m_cancelButton;
    private BooleanSupplier m_intakeCoralButton;
    private BooleanSupplier m_intakeHighButton;
    private BooleanSupplier m_scoreCoralButton;
    private BooleanSupplier m_regurgitateButton;
    private BooleanSupplier m_emergencyRetractButton;

    public static CoralSubsystem getInstance() {
        if (s_coralSubsystemInstance == null) {
            s_coralSubsystemInstance = new CoralSubsystem(CoralSubsystem.initializeHardware());
        }
        return s_coralSubsystemInstance;
    }

    public CoralSubsystem(Hardware hardware) {
        super(CoralSubsystemStates.AUTO);
        m_coralMotor = hardware.coralMotor;
        m_armMotor = hardware.armMotor;

        m_armMotorConfig = new SparkMaxConfig();
        m_armMotorConfig
            .idleMode(IdleMode.kBrake)
            .closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pid(Constants.CoralArmPID.P,
                    Constants.CoralArmPID.I,
                    Constants.CoralArmPID.D)
                .maxMotion
                    .allowedClosedLoopError(
                        Constants.CoralArmHardware.ALLOWED_CLOSED_LOOP_ERROR
                    );
        m_armController = m_armMotor.getClosedLoopController();
        m_armEncoder = m_armMotor.getEncoder();
        m_armMotorConfig.smartCurrentLimit((int)Constants.CoralArmHardware.ARM_MOTOR_CURRENT_LIMIT.in(Units.Amps));
        m_armMotor.configure(m_armMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_coralMotorConfig = new SparkMaxConfig();
        m_coralMotorConfig
            .idleMode(IdleMode.kBrake)
            .closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .ffUnits(ArbFFUnits.kVoltage)
                .pid(Constants.CoralArmPID.P,
                    Constants.CoralArmPID.I,
                    Constants.CoralArmPID.D)
                .maxMotion
                    .allowedClosedLoopError(
                        Constants.CoralArmHardware.ALLOWED_CLOSED_LOOP_ERROR
                    );
        m_coralMotorConfig.smartCurrentLimit((int)Constants.CoralArmHardware.ROLLER_MOTOR_CURRENT_LIMIT.in(Units.Amps));
        m_coralMotor.configure(m_coralMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        defaultState = CoralSubsystemState.NOTHING;
        defaultStateName = "Nothing";
    }

    public static Hardware initializeHardware() {
        Hardware coralSubsystemHardware = new Hardware(
            new SparkMax(Constants.CoralArmHardware.EFFECTOR_MOTOR_ID, MotorType.kBrushless),
            new SparkMax(Constants.CoralArmHardware.ARM_MOTOR_ID, MotorType.kBrushless)
        );
        return coralSubsystemHardware;
    }

}