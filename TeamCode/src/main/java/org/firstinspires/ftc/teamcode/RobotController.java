package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.IgnoreConfigurable;
import com.bylazar.panels.Panels;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.button.GamepadButton;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.factories.SpindexerFactory;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Spindexer;

public class RobotController extends CommandOpMode {

    private Drivetrain drivetrain;
    private Intake intake;
    private Spindexer spindexer;
    private Shooter shooter;

    private GamepadEx driverController;

    private GamepadButton homeSpindexer;

    @IgnoreConfigurable
    static TelemetryManager telemetryManager;

    @Override
    public void initialize() {
        telemetryManager = PanelsTelemetry.INSTANCE.getTelemetry();

        drivetrain = Drivetrain.getInstance(hardwareMap, telemetryManager);
        intake = Intake.getInstance(hardwareMap, telemetryManager);
        spindexer = Spindexer.getInstance(hardwareMap, telemetryManager);
        shooter = Shooter.getInstance(hardwareMap, telemetryManager);

        driverController = new GamepadEx(gamepad1);
        homeSpindexer = new GamepadButton(driverController, GamepadKeys.Button.A);

        homeSpindexer.whenPressed(SpindexerFactory.homeSpindexer(spindexer));

        register(drivetrain, intake, spindexer, shooter);
    }
}
