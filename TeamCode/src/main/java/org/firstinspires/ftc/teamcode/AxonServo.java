package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoControllerEx;
import com.qualcomm.robotcore.util.Range;

public class AxonServo {
    private static final double SERVO_UNUSABLE_DEGREES = 5.0;
    private static final double MIN_ANGLE = 0;
    private static final double MAX_ANGLE = 360 - SERVO_UNUSABLE_DEGREES;
    private static final double MIN_PWM = 500 + (2000.0 * 5.0/360.0); // 527.78
    private static final double MAX_PWM = 2500;
    private static final double TARGET_WINDOW_TOLERANCE = 5.0;

    private final Servo servo;
    private final AnalogInput servoEncoder;
    private final boolean reverse;
    private Double targetAngle;

    public AxonServo(Servo servo, AnalogInput servoEncoder, boolean reverse) {
        this.servo = servo;
        this.servoEncoder = servoEncoder;
        this.reverse = reverse;
        setServoRange();
    }

    private void setServoRange() {
        if(servo.getController() instanceof ServoControllerEx) {
            ServoControllerEx controller = (ServoControllerEx) servo.getController();
            controller.setServoPwmRange(
                    servo.getPortNumber(),
                    new PwmControl.PwmRange(MIN_PWM, MAX_PWM)
            );
        }
    }

    public void setTargetAngle(double angleDegrees) {
        this.targetAngle = Range.clip(angleDegrees, MIN_ANGLE, MAX_ANGLE);
        angleDegrees = reverse ? reverseAngle(angleDegrees) : angleDegrees;
        servo.setPosition(angleToServoPosition(angleDegrees));
    }

    private double reverseAngle(double angleDegrees) {
        double newAngle = (180 - angleDegrees) % 360;
        if (newAngle < 0) {
            newAngle += 360;
        }
        return newAngle;
    }

    public double getTargetAngle() {
        if (targetAngle == null) {
            return 0;
        }
        return targetAngle;
    }

    public boolean hasTargetAngle() {
        return (targetAngle != null);
    }

    public void adjustTargetAngle(double adjustmentInDegrees) {
        targetAngle += adjustmentInDegrees;
        targetAngle = Range.clip(targetAngle, MIN_ANGLE, MAX_ANGLE);
        servo.setPosition(angleToServoPosition(targetAngle));
    }

    public double getCurrentAngle() {
        if (servoEncoder == null) return 0.0;

        return ((1.0 - (servoEncoder.getVoltage() / servoEncoder.getMaxVoltage())) * 360.0) - SERVO_UNUSABLE_DEGREES;
    }

    public double getVoltage() {
        if (servoEncoder == null) return 0.0;

        return servoEncoder.getVoltage();
    }

    public double getRawMaxVoltage() {
        if (servoEncoder == null) return 0.0;

        return servoEncoder.getMaxVoltage();
    }

    public boolean isAtTarget() {
        if (targetAngle == null) {
            return true;
        }

        return (Math.abs(getCurrentAngle() - targetAngle) < TARGET_WINDOW_TOLERANCE);
    }

    private static double angleToServoPosition(double angleDegrees) {
        return Range.scale(angleDegrees, MIN_ANGLE, MAX_ANGLE, 0, 1);
    }
}
