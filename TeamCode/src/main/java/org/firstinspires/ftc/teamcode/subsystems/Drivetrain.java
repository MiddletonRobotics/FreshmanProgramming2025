package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.IgnoreConfigurable;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

public class Drivetrain extends SubsystemBase {
    private MotorEx frontLeft;
    private MotorEx frontRight;
    private MotorEx backLeft;
    private MotorEx backRight;

    private Follower follower;

    @IgnoreConfigurable
    static TelemetryManager telemetryManager;

    private static Drivetrain instance = null;
    public static synchronized Drivetrain getInstance(HardwareMap hMap, TelemetryManager telemetryManager) {
        if(instance == null) {
             instance = new Drivetrain(hMap, telemetryManager);
        }

        return instance;
    }

    private Drivetrain(HardwareMap hMap, TelemetryManager telemetryManager) {
        frontLeft = new MotorEx(hMap, "fl", Motor.GoBILDA.RPM_312);
        frontRight = new MotorEx(hMap, "fr", Motor.GoBILDA.RPM_312);
        backLeft = new MotorEx(hMap, "bl", Motor.GoBILDA.RPM_312);
        backRight = new MotorEx(hMap, "br", Motor.GoBILDA.RPM_312);

        follower = Constants.createFollower(hMap);
        follower.setStartingPose(new Pose(0,0,0));

        this.telemetryManager = telemetryManager;
    }

    public Pose getPose() {
        return follower.getPose();
    }

    public void resetPose(Pose pose) {
        follower.setPose(pose);
    }

    public void setMovementVectors(double forward, double strafe, double rotation) {
        follower.setTeleOpDrive(forward, strafe, rotation, false);
    }

    @Override
    public void periodic() {
        telemetryManager.debug("Drivetrain Pose X: " + getPose().getX());
        telemetryManager.debug("Drivetrain Pose Y: " + getPose().getY());
        telemetryManager.debug("Drivetrain Pose Rotation: " + getPose().getHeading() * (180 / Math.PI));
    }
}