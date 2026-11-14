
package org.firstinspires.ftc.teamcode.teleOps;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Drive Motor Tester", group = "Test")
public class DriveMotorTester extends OpMode {

    // Drive motors
    private DcMotorEx leftFront;
    private DcMotorEx leftBack;
    private DcMotorEx rightFront;
    private DcMotorEx rightBack;

    // Testing power
    private final double TEST_POWER = 0.25;

    @Override
    public void init() {

        // Map motors using PedroPathing hardware names
        leftFront = hardwareMap.get(DcMotorEx.class, "left_front");
        leftBack = hardwareMap.get(DcMotorEx.class, "left_back");
        rightFront = hardwareMap.get(DcMotorEx.class, "right_front");
        rightBack = hardwareMap.get(DcMotorEx.class, "right_back");

        // Match motor directions from PedroPathing Constants
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBack.setDirection(DcMotorSimple.Direction.FORWARD);

        // Configure motors
        DcMotorEx[] motors = {
                leftFront, leftBack, rightFront, rightBack
        };

        for (DcMotorEx motor : motors) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setPower(0);
        }

        telemetry.addLine("Drive Motor Tester Initialized");
        telemetry.addLine("A = Left Front");
        telemetry.addLine("B = Left Back");
        telemetry.addLine("X = Right Front");
        telemetry.addLine("Y = Right Back");
        telemetry.update();
    }

    @Override
    public void loop() {

        // Turn all motors off before applying new power
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);

        // Individual motor testing
        if (gamepad1.a) {
            leftFront.setPower(TEST_POWER);
        }

        if (gamepad1.b) {
            leftBack.setPower(TEST_POWER);
        }

        if (gamepad1.x) {
            rightFront.setPower(TEST_POWER);
        }

        if (gamepad1.y) {
            rightBack.setPower(TEST_POWER);
        }

        // Telemetry
        telemetry.addData("A - Left Front", gamepad1.a ? "ACTIVE" : "OFF");
        telemetry.addData("B - Left Back", gamepad1.b ? "ACTIVE" : "OFF");
        telemetry.addData("X - Right Front", gamepad1.x ? "ACTIVE" : "OFF");
        telemetry.addData("Y - Right Back", gamepad1.y ? "ACTIVE" : "OFF");

        telemetry.addData("Test Power", TEST_POWER);
        telemetry.update();
    }

    @Override
    public void stop() {

        // Stop all motors when OpMode ends
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
    }
}
