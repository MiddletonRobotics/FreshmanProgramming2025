package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.IgnoreConfigurable;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.ServoEx;

import org.firstinspires.ftc.teamcode.constants.SpindexerConstants;

public class Spindexer extends SubsystemBase {
    private ServoEx rotationServo;
    private AnalogInput rotationEncoder;

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
        rotationEncoder = hMap.get(AnalogInput.class, SpindexerConstants.kRotationServoEncoderID);

        colorSlot0 = hMap.get(RevColorSensorV3.class, SpindexerConstants.kSlot0ColorSensor);
        colorSlot1 = hMap.get(RevColorSensorV3.class, SpindexerConstants.kSlot1ColorSensor);
        colorSlot2 = hMap.get(RevColorSensorV3.class, SpindexerConstants.kSlot2ColorSensor);

        rightHomingSwitch = hMap.get(TouchSensor.class, SpindexerConstants.kRightHomingSwitch);
        leftHomingSwitch = hMap.get(TouchSensor.class, SpindexerConstants.kLeftHomingSwitch);

        this.telemetryManager = telemetryManager;
    }

    @Override
    public void periodic() {

    }
}