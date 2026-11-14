package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static Follower create(HardwareMap h) {
        return new Follower(
                new PinpointLocalizer(h, localizerConfig),
                new Mecanum(h, driveConfig),
                new Foresight(foresightConfig)
        );
    }

    public static MecanumConfig driveConfig = new MecanumConfig(
            c -> {
                c.frontLeftName.set("left_front");
                c.backLeftName.set("left_back");
                c.frontRightName.set("right_front");
                c.backRightName.set("right_back");

                c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
                c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
                c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
                c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
            }
    );
    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("pinpoint");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodOffset.set(2.9389361884650285);
        c.yPodOffset.set(-6.272490493894562);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });

    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.24860567857566004);
                Controller secondaryTranslationalForward = Controller.proportional(0.09185315536763142);
                Controller primaryTranslationalLateral = Controller.proportional(0.3274035174939583);
                Controller secondaryTranslationalLateral = Controller.proportional(0.12096685133090887);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.018495763871762234));
                c.brake.set(Controller.proportionalFeedforward(0.0157213992909979));

                c.headingFeedback.set(Controller.proportional(4.675942799094183));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.04176434327726595, 0.0049119543125505875));

                c.linearBrakeCoefficients.set(Matrix.diag(0.0661327037893612, 0.048757316293215225));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.0011584857421547648, 0.0015274061226195602));

                c.maxAchievableForwardVelocity.set(55.194351879525755);
                c.maxAchievableStrafeVelocity.set(43.263030894883286);
                c.naturalForwardDeceleration.set(33.7823423666487);
                c.naturalStrafeDeceleration.set(60.48434348884932);
            }
    );
}