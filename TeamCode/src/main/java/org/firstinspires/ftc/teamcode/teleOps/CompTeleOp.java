package org.firstinspires.ftc.teamcode.teleOps;

import com.pedropathing.controllers.PIDController;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.HiveTargeting;
import org.firstinspires.ftc.teamcode.ParkPoseStorage;
import org.firstinspires.ftc.teamcode.pedro.Constants;

@SuppressWarnings("unused")
@TeleOp
public class CompTeleOp extends OpMode {
    // default pose to avoid null errors
    private final PIDController headingController = new PIDController(2.0, 0.0, 0.1);
    private Follower follower;
    private HiveTargeting hiveTargeting;

    @Override
    public void init() {
        hiveTargeting = new HiveTargeting();
        follower = Constants.create(hardwareMap);
    }

    @Override
    public void start() {
        follower.setPose(ParkPoseStorage.parkPose);
        follower.update();
    }
    @Override
    public void loop() {
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
    }
}