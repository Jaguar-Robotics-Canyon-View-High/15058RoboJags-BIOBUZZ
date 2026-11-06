package org.firstinspires.ftc.teamcode;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class HiveTargeting {
    private static final double FIELD_CENTER = 72;
    private static final double HIVE_OFFSET_X = 12.75; // all hive buckets are 12.75” left/right from center field
    private static final double HIVE_OFFSET_Y = 19.75; // all hive buckets are 19.75” forward/back from center field
    private static final Pose hiveScoring = PoseFactory.degrees().of(FIELD_CENTER - HIVE_OFFSET_X, FIELD_CENTER + HIVE_OFFSET_Y, 0);
    private static final Pose hiveAudience = PoseFactory.degrees().of(FIELD_CENTER - HIVE_OFFSET_X, FIELD_CENTER - HIVE_OFFSET_Y, 0);
    private static final double TURN_KP = 1.0; // tune this
    private static final double MAX_TURN_POWER = 0.8;
    private static final double HEADING_TOLERANCE = Math.toRadians(1.5);
    private double lastFieldHeadingToTarget; // radians

    // Turn power (-MAX..+MAX) that rotates the robot's front toward the closest hive.
    public double calculateTurnPowerToClosestTarget(Pose robotPose) {
        Pose target = closestTarget(robotPose);
        lastFieldHeadingToTarget = Math.atan2(
                target.y() - robotPose.y(),
                target.x() - robotPose.x());

        double error = wrapAngle(lastFieldHeadingToTarget - robotPose.heading());
        if (Math.abs(error) < HEADING_TOLERANCE) {
            return 0;
        }
        double power = TURN_KP * error;
        return Math.max(-MAX_TURN_POWER, Math.min(MAX_TURN_POWER, power));
    }

    public void outputTelemetry(Telemetry telemetry) {
        telemetry.addData("Target Field Heading (deg)", Math.toDegrees(lastFieldHeadingToTarget));
    }

    private Pose closestTarget(Pose robotPose) {
        double scoring = calculateDistanceToTarget(robotPose, hiveScoring);
        double audience = calculateDistanceToTarget(robotPose, hiveAudience);
        return (audience < scoring) ? hiveAudience : hiveScoring;
    }

    private double calculateDistanceToTarget(Pose robotPose, Pose targetPose) {
        return Math.hypot(targetPose.x() - robotPose.x(), targetPose.y() - robotPose.y());
    }

    private static double wrapAngle(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }
}