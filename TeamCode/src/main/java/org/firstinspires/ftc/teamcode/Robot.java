package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.seattlesolvers.solverslib.controller.PIDFController;

import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;

@Autonomous(name="heuihfieuwhfiuewhfiues")
public class Robot extends OpMode {
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    private DcMotor elevatorMotor;
    private RevTouchSensor homingSwitch;
    private Drivetrain drivetrain;
    private PIDFController pidController;

    double targetPosition = 0;

    @Override
    public void init() {
        drivetrain = Drivetrain.getInstance(hardwareMap);
        frontLeft = hardwareMap.get(DcMotor.class, "fl");
        frontRight = hardwareMap.get(DcMotor.class, "fr");
        backLeft = hardwareMap.get(DcMotor.class, "bl");
        backRight = hardwareMap.get(DcMotor.class, "br");
        elevatorMotor = hardwareMap.get(DcMotor.class, "em");
        homingSwitch = hardwareMap.get(RevTouchSensor.class, "hs");

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        elevatorMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        elevatorMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        pidController = new PIDFController(0.001,0,0,0);
    }

    @Override
    public void loop() {
        double currentPosition = elevatorMotor.getCurrentPosition();

        double forward = gamepad1.left_stick_y;
        double strafe = -gamepad1.left_stick_x;
        double rotation = gamepad1.right_stick_x;

        telemetry.addData("Current Position", currentPosition);
        telemetry.addData("Target Position", targetPosition);
        telemetry.update();

        double[] speeds = new double[] {
                (forward + rotation + strafe),
                (forward - rotation - strafe),
                (forward + rotation - strafe),
                (forward - rotation + strafe)
        };

        if(gamepad1.a) {
            targetPosition = 1000;
        }

        elevatorMotor.setPower(pidController.calculate(currentPosition, targetPosition));

        frontLeft.setPower(speeds[0]);
        frontRight.setPower(speeds[1]);
        backLeft.setPower(speeds[2]);
        backRight.setPower(speeds[3]);
    }
}
