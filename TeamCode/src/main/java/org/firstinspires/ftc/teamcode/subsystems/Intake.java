package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.IgnoreConfigurable;
import com.bylazar.telemetry.TelemetryManager;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.controller.wpilibcontroller.SimpleMotorFeedforward;
import com.seattlesolvers.solverslib.hardware.motors.Motor;

import org.firstinspires.ftc.teamcode.constants.IntakeConstants;
import org.firstinspires.ftc.teamcode.utilties.BallType;

public class Intake extends SubsystemBase {
    private Motor intakeMotor;
    private Motor.Encoder intakeEncoder;
    private DigitalChannel beamBreak;
    private RevColorSensorV3 colorSensor;

    @IgnoreConfigurable
    static TelemetryManager telemetryManager;

    private PIDFController pidfController;
    private SimpleMotorFeedforward feedforward;

    private double intakeGearRatio;

    private static Intake instance;
    public static synchronized Intake getInstance(HardwareMap hMap, TelemetryManager telemetryManager) {
        if(instance == null) {
            instance = new Intake(hMap, telemetryManager);
        }

        return instance;
    }

    private Intake(HardwareMap hMap, TelemetryManager telemetryManager) {
        intakeMotor = new Motor(hMap, IntakeConstants.intakeMotorID, Motor.GoBILDA.RPM_1150);
        intakeEncoder = intakeMotor.encoder;
        beamBreak = hMap.get(DigitalChannel.class, IntakeConstants.intakeBeamBreakID);
        colorSensor = hMap.get(RevColorSensorV3.class, IntakeConstants.intakeColorSensorID);

        intakeGearRatio = (48 * 60);

        pidfController = new PIDFController(IntakeConstants.P, IntakeConstants.I, IntakeConstants.D, IntakeConstants.F);
        feedforward = new SimpleMotorFeedforward(0.0, 0.0, 0.0);

        intakeMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        intakeMotor.setInverted(true);

        this.telemetryManager = telemetryManager;
    }

    public double getVelocity() {
        return (intakeEncoder.getCorrectedVelocity() / intakeMotor.getCPR()) * (1 / intakeGearRatio);
    }

    public boolean isBallPurple() {
        int colorR = colorSensor.red();
        int colorG = colorSensor.green();
        int colorB = colorSensor.blue();

        return (Math.abs(colorR - BallType.PURPLE.getR()) <= 10) || (Math.abs(colorG - BallType.PURPLE.getG()) <= 10) || (Math.abs(colorB - BallType.PURPLE.getB()) <= 10);
    }

    public boolean isLoaded() {
        return !beamBreak.getState();
    }


    public void setFlywheelRPM(double desiredRPM) {
        double output = pidfController.calculate(getVelocity(), desiredRPM);
        double ff = feedforward.calculate(getVelocity());

        intakeMotor.set(output + ff);
    }

    @Override
    public void periodic() {
        telemetryManager.addData("Intake Velocity", getVelocity());
        telemetryManager.addData("Intake RPM Setpoint", pidfController.getSetPoint());
        telemetryManager.addData("Intake BeamBreak Status", isLoaded());
    }
}