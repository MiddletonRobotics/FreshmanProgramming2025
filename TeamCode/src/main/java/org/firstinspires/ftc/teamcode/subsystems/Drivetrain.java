package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.IgnoreConfigurable;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

import org.firstinspires.ftc.library.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

public class Drivetrain extends SubsystemBase {
    private MotorEx frontLeft;
    private MotorEx frontRight;
    private MotorEx backLeft;
    private MotorEx backRight;

    private Follower follower;

    public enum WantedState {
        TELEOP_DRIVE,
        PEDROPATHING_PATH,
        ROTATION_LOCK,
        DRIVE_TO_POINT,
        ON_THE_FLY,
        IDLE
    }

    private enum SystemState {
        TELEOP_DRIVE,
        PEDROPATHING_PATH,
        ROTATION_LOCK,
        DRIVE_TO_POINT,
        ON_THE_FLY,
        IDLE
    }

    private WantedState wantedState = WantedState.TELEOP_DRIVE;
    private SystemState systemState = SystemState.TELEOP_DRIVE;

    private final PIDFController headingController;
    private final PIDFController teleopDriveToPointController;
    private final PIDFController autonomousDriveToPointController;

    private double desiredHeadingRadians;
    private double maxVelocityOutputForDriveToPoint;
    private Pose2d desiredPoseForDriveToPoint = new Pose2d();

    private double forward = 0.0;
    private double strafe = 0.0;
    private double rotation = 0.0;
    private boolean robotCentric = false;

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

        this.headingController = new PIDFController(0.01, 0, 0, 0);
        this.teleopDriveToPointController = new PIDFController(0.01, 0, 0, 0);
        this.autonomousDriveToPointController = new PIDFController(0.01, 0, 0, 0);

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