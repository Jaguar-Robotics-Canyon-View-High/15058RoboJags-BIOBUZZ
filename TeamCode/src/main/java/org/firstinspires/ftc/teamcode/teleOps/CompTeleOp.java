package org.firstinspires.ftc.teamcode.teleOps;

import com.pedropathing.controllers.PIDController;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.HiveTargeting;
import org.firstinspires.ftc.teamcode.ParkPoseStorage;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.OtherHardware;

@SuppressWarnings("unused")
@TeleOp
public class CompTeleOp extends OpMode {
    private final PIDController headingController = new PIDController(2.0, 0.0, 0.1);
    private Follower follower;
    private HiveTargeting hiveTargeting;
    private OtherHardware otherHardware;
    private double shooterPower = 0.9;

    @Override
    public void init() {
        hiveTargeting = new HiveTargeting();
        otherHardware = new OtherHardware();
        follower = Constants.create(hardwareMap);
        otherHardware.init(hardwareMap);
    }

    @Override
    public void start() {
        follower.setPose(ParkPoseStorage.parkPose);
        follower.update();
    }
    @Override
    public void loop() {
        final double SHOOTER_INCREMENT = 0.05;
        double intakePower;
        // Intake Motor logic
        if (gamepad1.left_trigger_pressed) {
            intakePower = 1.0;
        } else if (gamepad1.left_bumper) {
            intakePower = -0.5;
        } else {
            intakePower = 0.0;
        }
        // Flywheel/Shooter logic to test different shooter values
        if (gamepad1.dpadUpWasPressed()) {
            shooterPower += SHOOTER_INCREMENT;
        }
        if (gamepad1.dpadDownWasPressed()) {
            shooterPower -= SHOOTER_INCREMENT;
        }
        // Keeps the shooter power between 0 and 1. Assigns right trigger to shooter power
        shooterPower = Range.clip(shooterPower, 0.0, 1.0);
        double activeShooterPower = gamepad1.right_trigger_pressed ? shooterPower : 0.0;
        // Sets power to all affiliated motors
        otherHardware.shooter.setPower(activeShooterPower);
        otherHardware.bottomIntake.setPower(intakePower);
        otherHardware.topIntake.setPower(intakePower);

        Pose robotPose = follower.pose();

        DrivePowers powers = new DrivePowers(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);
        if (gamepad1.a) {
            double targetHeading = hiveTargeting.calculateAngleToClosestTarget(robotPose);
            powers = ManualDrive.headingLock(follower, headingController, powers, targetHeading);
        }

        powers = ManualDrive.fieldCentric(powers, robotPose.heading());

        ManualDrive.driveOrHold(follower, powers);
        follower.update();

        hiveTargeting.outputTelemetry(telemetry);
        telemetry.addData("Robot X", robotPose.x());
        telemetry.addData("Robot Y", robotPose.y());
        telemetry.addData("Robot Heading", Math.toDegrees(robotPose.heading()));
        telemetry.addData("Shooter Enabled", gamepad1.right_trigger_pressed);
        telemetry.addData("Shooter Power", shooterPower);
    }
}