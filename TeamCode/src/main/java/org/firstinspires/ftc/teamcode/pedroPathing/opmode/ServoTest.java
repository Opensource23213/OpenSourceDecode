package org.firstinspires.ftc.teamcode.pedroPathing.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@Disabled
@TeleOp(name="Servo Test", group="ABC Opmode")

public class ServoTest extends DecodeLibrary{
    public Servo servo1;
    public static double test_pos = 0;
    @Override
    public void init(){
        servo1 = hardwareMap.get(Servo.class, "flippy");
    }
    @Override
    public void loop(){
        servo1.setPosition(test_pos);
        telemetry.addData("Servo Position", test_pos);
        telemetry.update();
    }

}
