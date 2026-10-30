package org.firstinspires.ftc.teamcode;


import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;


import org.firstinspires.ftc.teamcode.pedro.Constants;


@SuppressWarnings("unused")
@TeleOp
public class HiveFinderTestOpMode1 extends OpMode {
    private static final Pose startPose = PoseFactory.degrees().of(132, 9, 90);
    private static final double FIELD_CENTER = 72;
    private static final double HIVE_OFFSET_X = 12.75; // all hive buckets are 12.75” left/right from center field
    private static final double HIVE_OFFSET_Y = 19.75; // all hive buckets are 19.75” forward/back from center field
    private static final Pose hiveBlueAudience = PoseFactory.degrees().of(FIELD_CENTER + HIVE_OFFSET_X, FIELD_CENTER - HIVE_OFFSET_Y, 0);
    private static final Pose hiveBlueScoring = PoseFactory.degrees().of(FIELD_CENTER + HIVE_OFFSET_X, FIELD_CENTER + HIVE_OFFSET_Y, 0);
    private Follower follower;
    private AxonServo servo;

    @Override
    public void init() {
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
        double hiveBlueAudienceDistance = calculateDistanceToTarget(robotPose, hiveBlueAudience);
        double hiveBlueScoringDistance = calculateDistanceToTarget(robotPose, hiveBlueScoring);

        Pose target;
        if (hiveBlueScoringDistance < hiveBlueAudienceDistance) {
            target = hiveBlueScoring;
        }
        else {
            target = hiveBlueAudience;
        }
        double angleToTarget = calculateAngleToTarget(robotPose, target);
        telemetry.addData("Target Heading", angleToTarget);
        servo.setTargetAngle(angleToTarget);
    }

    private double calculateAngleToTarget(Pose robotPose, Pose targetPose) {
        // Find the absolute heading (angle) of our target using atan2
        double deltaX = targetPose.x() - robotPose.x();
        double deltaY = targetPose.y() - robotPose.y();
        double absoluteTargetHeading = Math.atan2(deltaY, deltaX);


        // Find the relative target heading from our robot heading
        double relativeTargetHeading = absoluteTargetHeading - robotPose.heading();


        // Wrap relative heading between -180 and +180
        while (relativeTargetHeading > Math.PI) relativeTargetHeading -= 2 * Math.PI;
        while (relativeTargetHeading < -Math.PI) relativeTargetHeading += 2 * Math.PI;


        // Add 90 degrees because forward on our turret is 90 degrees
        double turretAngleRad = (Math.PI / 2.0) + relativeTargetHeading;


        // Keep the final turret angle between 0 and 360 degrees
        double TWO_PI = 2.0 * Math.PI;
        turretAngleRad = (turretAngleRad % TWO_PI + TWO_PI) % TWO_PI;


        return Math.toDegrees(turretAngleRad);

    }
    private double calculateDistanceToTarget(Pose robotPose, Pose targetPose) {
        // The distance is just the hypotenuse of our right triangle
        return Math.hypot(targetPose.x() - robotPose.x(), targetPose.y() - robotPose.y());
    }

}
