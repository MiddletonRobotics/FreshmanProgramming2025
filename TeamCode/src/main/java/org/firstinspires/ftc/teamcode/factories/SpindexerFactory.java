package org.firstinspires.ftc.teamcode.factories;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.seattlesolvers.solverslib.command.Command;

import org.firstinspires.ftc.library.commands.Commands;
import org.firstinspires.ftc.teamcode.subsystems.Spindexer;

public class SpindexerFactory {
    public static Command homeSpindexer(Spindexer spindexer) {
        return Commands.run(() -> {
            boolean isSpindexerHomed;
            boolean isSpindexerHoming;

            if(!spindexer.isRightSwitchTriggered() && !spindexer.isLeftSwitchTriggered()) {
                isSpindexerHomed = false;
                isSpindexerHoming = true;

                spindexer.setSpeed(0.5);
            } else if (spindexer.isRightSwitchTriggered() && spindexer.isLeftSwitchTriggered()) {
                spindexer.setSpeed(0.0);
                isSpindexerHomed = true;
            }
        }, spindexer);
    }
}
