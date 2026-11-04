package org.firstinspires.ftc.teamcode;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedro.Constants;

@TeleOp
public class CompTeleOp extends OpMode {
    private static final Pose startPose = PoseFactory.degrees().of(132, 9, 90);


    private Alliance allianceColor = Alliance.NONE;
    private Follower follower;
    private HiveTargeting hiveTargeting;
    @Override
    public void init() {
        hiveTargeting = new HiveTargeting();
        follower = Constants.create(hardwareMap);

        telemetry.addData("Status", "Initialized. Use Gamepad 1 to select Alliance");
        telemetry.addData("Selection", "X = BLUE, A = RED");
    }
    @Override
    public void init_loop() {
        if (gamepad1.x) {
            allianceColor = Alliance.BLUE;
        } else if (gamepad1.a) {
            allianceColor = Alliance.RED;
        }
        telemetry.addData("Selected Alliance", allianceColor);
        telemetry.update();
    }
    @Override
    public void start() {telemetry.addData("Match Started! Alliance locked", allianceColor);}
    @Override
    public void loop() {
        double robotHeading = follower.pose().heading();
        if (gamepad1.a){
            robotHeading = hiveTargeting.calculateAngleToClosestTarget(follower.pose(),allianceColor);
        }
        DrivePowers powers = ManualDrive.fieldCentric(
                -gamepad1.left_stick_y,
                gamepad1.left_stick_x,
                gamepad1.right_stick_x,
                robotHeading
        );
        ManualDrive.driveOrHold(follower, powers);
        follower.update();
        hiveTargeting.outputTelementry(telemetry);

        Pose robotPose = follower.pose();
        telemetry.addData("Robot X", robotPose.x());
        telemetry.addData("Robot Y", robotPose.y());
        telemetry.addData("Robot Heading", Math.toDegrees(robotPose.heading()));
    }
}