package org.firstinspires.ftc.teamcode;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Vector2D;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class HiveTargeting {
    private static final double FIELD_CENTER = 72;
    private static final double HIVE_OFFSET_X = 12.75; // all hive buckets are 12.75” left/right from center field
    private static final double HIVE_OFFSET_Y = 19.75; // all hive buckets are 19.75” forward/back from center field
    private static final Pose hiveScoring = PoseFactory.degrees().of(FIELD_CENTER - HIVE_OFFSET_X, FIELD_CENTER + HIVE_OFFSET_Y, 0);
    private static final Pose hiveAudience = PoseFactory.degrees().of(FIELD_CENTER - HIVE_OFFSET_X, FIELD_CENTER - HIVE_OFFSET_Y, 0);
    private double lastAngleToTarget;

    public double calculateAngleToClosestTarget(Pose robotPose) {
        Pose target = closestTarget(robotPose);
        lastAngleToTarget = calculateAngleToTarget(robotPose, target);
        return lastAngleToTarget;
    }

    private Pose closestTarget(Pose robotPose) {
        double scoring = calculateDistanceToTarget(robotPose, hiveScoring);
        double audience = calculateDistanceToTarget(robotPose, hiveAudience);
        return (audience < scoring) ? hiveAudience : hiveScoring;
    }

    public void outputTelemetry(Telemetry telemetry) {
        telemetry.addData("Target Heading", lastAngleToTarget);
    }

    private double calculateAngleToTarget(Pose robotPose, Pose targetPose) {
        Vector2D toHive = targetPose.toVector2D().minus(robotPose.toVector2D());
        return toHive.theta();
    }

    private double calculateDistanceToTarget(Pose robotPose, Pose targetPose) {
        // The distance is just the hypotenuse of our right triangle
        return Math.hypot(targetPose.x() - robotPose.x(), targetPose.y() - robotPose.y());
    }
}
