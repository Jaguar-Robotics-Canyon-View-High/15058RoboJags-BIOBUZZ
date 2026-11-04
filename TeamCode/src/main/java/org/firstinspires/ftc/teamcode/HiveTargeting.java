package org.firstinspires.ftc.teamcode;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class HiveTargeting {
    private static final double FIELD_CENTER = 72;
    private static final double HIVE_OFFSET_X = 12.75; // all hive buckets are 12.75” left/right from center field
    private static final double HIVE_OFFSET_Y = 19.75; // all hive buckets are 19.75” forward/back from center field
    private static final Pose hiveBlueAudience = PoseFactory.degrees().of(FIELD_CENTER + HIVE_OFFSET_X, FIELD_CENTER - HIVE_OFFSET_Y, 0);
    private static final Pose hiveBlueScoring = PoseFactory.degrees().of(FIELD_CENTER + HIVE_OFFSET_X, FIELD_CENTER + HIVE_OFFSET_Y, 0);
    private static final Pose hiveRedScoring = PoseFactory.degrees().of(FIELD_CENTER - HIVE_OFFSET_X, FIELD_CENTER + HIVE_OFFSET_Y, 0);
    private static final Pose hiveRedAudience = PoseFactory.degrees().of(FIELD_CENTER - HIVE_OFFSET_X, FIELD_CENTER - HIVE_OFFSET_Y, 0);
    private double lastAngleToTarget;

    public double calculateAngleToClosestTarget(Pose robotPose, Alliance alliance) {
        double hiveBlueAudienceDistance = calculateDistanceToTarget(robotPose, hiveBlueAudience);
        double hiveBlueScoringDistance = calculateDistanceToTarget(robotPose, hiveBlueScoring);
        double hiveRedScoringDistance = calculateDistanceToTarget(robotPose, hiveRedScoring);
        double hiveRedAudienceDistance = calculateDistanceToTarget(robotPose, hiveRedAudience);

        Pose target = null;
        if (alliance == Alliance.BLUE) {
            if (hiveBlueScoringDistance < hiveBlueAudienceDistance) {
                target = hiveBlueScoring;
            } else {
                target = hiveBlueAudience;
            }
        } else if (alliance == Alliance.RED) {
            if (hiveRedAudienceDistance < hiveRedScoringDistance) {
                target = hiveRedAudience;
            } else {
                target = hiveRedScoring;
            }
        }
        lastAngleToTarget = calculateAngleToTarget(robotPose, target);
        return lastAngleToTarget;
    }

    public void outputTelementry(Telemetry telemetry) {
        telemetry.addData("Target Heading", lastAngleToTarget);
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
