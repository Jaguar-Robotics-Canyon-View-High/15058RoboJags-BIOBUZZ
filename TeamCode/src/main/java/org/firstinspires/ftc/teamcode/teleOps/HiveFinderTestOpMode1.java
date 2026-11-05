package org.firstinspires.ftc.teamcode.teleOps;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Alliance;
import org.firstinspires.ftc.teamcode.AxonServo;
import org.firstinspires.ftc.teamcode.HiveTargeting;
import org.firstinspires.ftc.teamcode.ParkPoseStorage;
import org.firstinspires.ftc.teamcode.pedro.Constants;


@SuppressWarnings("unused")
@TeleOp
public class HiveFinderTestOpMode1 extends OpMode {
    // default pose to avoid null errors
    private static final Pose startPose = PoseFactory.degrees().of(0, 0, 0);
    private HiveTargeting hiveTargeting;
    private Follower follower;
    private AxonServo servo;

    @Override
    public void start() {
        follower.setPose(ParkPoseStorage.parkPose);
        follower.update();
    }
    @Override
    public void init() {
        hiveTargeting = new HiveTargeting();
        follower = Constants.create(hardwareMap);
        follower.setPose(startPose);
        servo = new AxonServo(hardwareMap.get(Servo.class, "pointerServo"), null, true);
    }


    @Override
    public void loop() {
        follower.manual(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x
        );
        follower.update();

        if (gamepad1.dpadUpWasPressed()) {
            servo.setTargetAngle(90);
        }
        if (gamepad1.dpadLeftWasPressed()) {
            servo.setTargetAngle(180);
        }
        if (gamepad1.dpadDownWasPressed()) {
            servo.setTargetAngle(270);
        }
        if (gamepad1.dpadRightWasPressed()) {
            servo.setTargetAngle(0);
        }

        Pose robotPose = follower.pose();
        telemetry.addData("Robot X", robotPose.x());
        telemetry.addData("Robot Y", robotPose.y());
        telemetry.addData("Robot Heading", Math.toDegrees(robotPose.heading()));
        double angleToTarget = hiveTargeting.calculateAngleToClosestTarget(robotPose, Alliance.RED);
        hiveTargeting.outputTelemetry(telemetry);
        servo.setTargetAngle(angleToTarget);
    }

}
