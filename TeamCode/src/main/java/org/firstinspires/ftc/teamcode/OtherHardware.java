package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class OtherHardware {
    // Motors
    public DcMotor bottomIntake;
    public DcMotor topIntake;
    public DcMotor shooter;

    public void init(HardwareMap hardwareMap) {
        bottomIntake = hardwareMap.get(DcMotor.class, "bottomIntake");
        topIntake = hardwareMap.get(DcMotor.class, "topIntake");
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
    }
}
