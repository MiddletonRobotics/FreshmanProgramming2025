package org.firstinspires.ftc.teamcode.constants;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class SpindexerConstants {
    public static final String kRotationServoID = "rs";
    public static final String kRotationServoEncoderID = "rse";
    public static final String kSlot0ColorSensor = "rsEe";
    public static final String kSlot1ColorSensor = "rsed";
    public static final String kSlot2ColorSensor = "rses";
    public static final String kRightHomingSwitch = "rsea";
    public static final String kLeftHomingSwitch = "rsSe";

    public static double kSlot0LoadingPosition = 0.0;
    public static double kSlot1LoadingPosition = 0.33;
    public static double kSlot2LoadingPosition = 0.66;

    public static double kSlot0ShootingPosition = 0.5;
    public static double kSlot1ShootingPosition = 0.5 - 0.33;
    public static double kSlot2ShootingPosition = 0.5 + 0.33;

}
