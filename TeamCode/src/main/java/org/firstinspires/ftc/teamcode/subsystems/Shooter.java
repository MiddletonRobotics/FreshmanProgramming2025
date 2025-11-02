package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.telemetry.TelemetryManager;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.controller.wpilibcontroller.SimpleMotorFeedforward;
import com.seattlesolvers.solverslib.hardware.motors.CRServoEx;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Shooter extends SubsystemBase {
    private MotorEx leftShooter;
    private MotorEx rightShooter;

    private CRServoEx leftHood;
    private CRServoEx rightHood;

    private DigitalChannel hoodLimitSwitch;

    private PIDFController flywheelPidfController, hoodPidfController;
    private SimpleMotorFeedforward feedforward;

    private double flywheelTarget, hoodTarget, hoodHomingPosition;
    private boolean hoodIsHomed = true;
    private boolean hoodIsHoming = false;

    private static Shooter instance;
    public static synchronized Shooter getInstance(HardwareMap hMap, TelemetryManager telemetryManager) {
        if(instance == null) {
            instance = new Shooter(hMap, telemetryManager);
        }

        return instance;
    }

    private Shooter(HardwareMap hMap, TelemetryManager telemetryManager) {
        leftShooter = new MotorEx(hMap, "ls", Motor.GoBILDA.BARE);
        rightShooter = new MotorEx(hMap, "rs", Motor.GoBILDA.BARE);

        leftHood = new CRServoEx(hMap, "lh");
        rightHood = new CRServoEx(hMap, "rh");

        hoodLimitSwitch = hMap.get(DigitalChannel.class, "hls");

        flywheelPidfController = new PIDFController(0.01,0,0,0);
        hoodPidfController = new PIDFController(0.01,0,0,0);
        feedforward = new SimpleMotorFeedforward(0,0,0);

        leftShooter.setInverted(true);
        rightShooter.setInverted(false);

        leftHood.setInverted(true);
        rightHood.setInverted(false);

        leftShooter.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        rightShooter.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
    }

    public double getVelocity() {
        return (leftShooter.getVelocity() + rightShooter.getVelocity()) / 2;
    }

    public double getHoodPosition() {
        return leftHood.getCurrentPosition();
    }

    public void homingSequence() {
        if(!hoodLimitSwitch.getState()) hoodIsHomed = false;
        if(!hoodIsHomed && !hoodLimitSwitch.getState()) {
            hoodIsHoming = true;
            leftHood.set(-0.2);
            rightHood.set(-0.2);
        } else {
            hoodIsHoming = false;
            leftHood.set(0);
            rightHood.set(0);

            hoodHomingPosition = getHoodPosition();
        }
    }

    public void setFlywheelTarget(double distance) {
        if(distance < 48 && distance >= 0) {
            flywheelTarget = (800 * distance) - 589;
        } else if(distance >= 48 && distance < 84) {
            flywheelTarget = (750 * distance) - 600;
        } else {
            flywheelTarget = (900 * distance) - 430;
        }
    }

    public void setHoodTarget(double degrees) {
        double zeroPosition = 66;
        hoodTarget = (zeroPosition - degrees) / 355;
    }

    @Override
    public void periodic() {
        leftHood.set(hoodPidfController.calculate(hoodTarget, getHoodPosition() - hoodHomingPosition));
        rightHood.set(hoodPidfController.calculate(hoodTarget, getHoodPosition() - hoodHomingPosition));

        double output = flywheelPidfController.calculate(getVelocity(), flywheelTarget);
        double feedforwardOutput = feedforward.calculate(flywheelTarget);

        leftShooter.set(output + feedforwardOutput);
        rightShooter.set(output + feedforwardOutput);
    }
}
