package org.firstinspires.ftc.teamcode.autos;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import org.firstinspires.ftc.teamcode.pedro.Constants;

@Autonomous
public class farParkAuto extends OpMode{
    private Follower follower;
    private final PoseFactory poseFactory = PoseFactory.degrees();
    // Poses
    private final Pose startPose = poseFactory.of(9.5, 9.5, 90);
    private final Pose parkPose = poseFactory.of(9.5, 96, 0);
    private final Pose parkControl1 = poseFactory.of(34.75, 45.65, 0);

    // Path Methods
    private Path park() {
        return curve(startPose, parkControl1, parkPose).linear(startPose, parkPose);
    }
    private Command autoRoutine()  {
        return sequential(
                follow(follower, park())
        );
    }
    @Override
    public void start() {
        schedule(autoRoutine());
    }

    @Override
    public void init() {
        Scheduler.reset();

        follower = Constants.create(hardwareMap);
        follower.setPose(startPose);
        follower.update();
    }

    @Override
    public void loop() {
        follower.update();
        Scheduler.execute();
        telemetry.addData("X", follower.pose().x());
        telemetry.addData("Y", follower.pose().y());
        telemetry.addData("Heading", Math.toDegrees(follower.pose().heading()));
        telemetry.addData("Follower Mode", follower.mode());
        telemetry.update();
    }
}

