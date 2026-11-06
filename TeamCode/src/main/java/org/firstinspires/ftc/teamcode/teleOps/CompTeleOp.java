package org.firstinspires.ftc.teamcode.teleOps;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.HiveTargeting;
import org.firstinspires.ftc.teamcode.ParkPoseStorage;
import org.firstinspires.ftc.teamcode.pedro.Constants;

@TeleOp
public class CompTeleOp extends OpMode {
    // default pose to avoid null errors
    private static final Pose startPose = PoseFactory.degrees().of(0, 0, 0);
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

        double turn = -gamepad1.right_stick_x;   // normal driver rotation
        if (gamepad1.a) {
            turn = hiveTargeting.calculateTurnPowerToClosestTarget(robotPose);
        }

        DrivePowers powers = ManualDrive.fieldCentric(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                turn,
                robotPose.heading()
        );
        ManualDrive.driveOrHold(follower, powers);
        follower.update();

        hiveTargeting.outputTelemetry(telemetry);
        telemetry.addData("Robot X", robotPose.x());
        telemetry.addData("Robot Y", robotPose.y());
        telemetry.addData("Robot Heading", Math.toDegrees(robotPose.heading()));
    }
}