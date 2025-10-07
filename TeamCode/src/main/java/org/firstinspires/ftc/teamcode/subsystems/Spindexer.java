package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.IgnoreConfigurable;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.ServoEx;

import org.firstinspires.ftc.library.geometry.Rotation2d;
import org.firstinspires.ftc.library.hardware.AnalogEncoder;
import org.firstinspires.ftc.teamcode.constants.SpindexerConstants;
import org.firstinspires.ftc.teamcode.utilties.BallType;

public class Spindexer extends SubsystemBase {
    private ServoEx rotationServo;
    private AnalogEncoder rotationEncoder;

    private RevColorSensorV3 colorSlot0;
    private RevColorSensorV3 colorSlot1;
    private RevColorSensorV3 colorSlot2;

    private TouchSensor rightHomingSwitch;
    private TouchSensor leftHomingSwitch;

    @IgnoreConfigurable
    static TelemetryManager telemetryManager;

    private static Spindexer instance;
    private static synchronized Spindexer getInstance(HardwareMap hMap, TelemetryManager telemetryManager) {
        if (instance == null) {
            instance = new Spindexer(hMap, telemetryManager);
        }

        return instance;
    }

    private Spindexer(HardwareMap hMap, TelemetryManager telemetryManager) {
        rotationServo = new ServoEx(hMap, SpindexerConstants.kRotationServoID);
        rotationEncoder = new AnalogEncoder(hMap, SpindexerConstants.kRotationServoEncoderID);

        colorSlot0 = hMap.get(RevColorSensorV3.class, SpindexerConstants.kSlot0ColorSensor);
        colorSlot1 = hMap.get(RevColorSensorV3.class, SpindexerConstants.kSlot1ColorSensor);
        colorSlot2 = hMap.get(RevColorSensorV3.class, SpindexerConstants.kSlot2ColorSensor);

        rightHomingSwitch = hMap.get(TouchSensor.class, SpindexerConstants.kRightHomingSwitch);
        leftHomingSwitch = hMap.get(TouchSensor.class, SpindexerConstants.kLeftHomingSwitch);

        this.telemetryManager = telemetryManager;
    }

    public boolean isBallPurple(RevColorSensorV3 colorSensor) {
        int red = colorSensor.red();
        int green = colorSensor.green();
        int blue = colorSensor.blue();

        return (Math.abs(red - BallType.PURPLE.getR()) <= 10) || (Math.abs(green - BallType.PURPLE.getG()) <= 10) || (Math.abs(blue - BallType.PURPLE.getB()) <= 10);
    }

    public boolean isBallPresent(RevColorSensorV3 colorSensor) {
        int red = colorSensor.red();
        int green = colorSensor.green();
        int blue = colorSensor.blue();

        return red == 0 && green == 0 && blue == 0;
    }

    public boolean isRightSwitchTriggered() {
        return rightHomingSwitch.isPressed();
    }

    public boolean isLeftSwitchTriggered() {
        return leftHomingSwitch.isPressed();
    }

    public Rotation2d getServoEncoderPosition() {
        return rotationEncoder.getAngle();
    }

    public void setPosition(double position) {
        rotationServo.set(position);
    }

    public void toIntakingPosition(int slotNumber) {
        switch (slotNumber) {
            case 0:
                rotationServo.set(SpindexerConstants.kSlot0LoadingPosition);
                break;
            case 1:
                rotationServo.set(SpindexerConstants.kSlot1LoadingPosition);
                break;
            case 2:
                rotationServo.set(SpindexerConstants.kSlot2LoadingPosition);
                break;
            default:
                throw new RuntimeException("Invalid slot number");
        }
    }

    public void toOuttakingPosition(int slotNumber) {
        switch (slotNumber) {
            case 0:
                rotationServo.set(SpindexerConstants.kSlot0ShootingPosition);
                break;
            case 1:
                rotationServo.set(SpindexerConstants.kSlot1ShootingPosition);
                break;
            case 2:
                rotationServo.set(SpindexerConstants.kSlot2ShootingPosition);
                break;
            default:
                throw new RuntimeException("Invalid slot number");
        }
    }

    @Override
    public void periodic() {

    }
}