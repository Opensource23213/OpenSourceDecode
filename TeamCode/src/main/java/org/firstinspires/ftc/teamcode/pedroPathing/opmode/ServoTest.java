package org.firstinspires.ftc.teamcode.pedroPathing.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp(name="Servo Test", group="ABC Opmode")

public class ServoTest extends DecodeLibrary{
    public Servo servo1;
    public Servo servo2;
    public static double test_pos = .5;
    public static double offset = .0075;
    @Override
    public void init(){
        servo1 = hardwareMap.get(Servo.class, "turret_servo_1");
        servo2 = hardwareMap.get(Servo.class, "turret_servo_2");
    }
    @Override
    public void loop(){
        servo1.setPosition(test_pos + offset);
        servo2.setPosition(test_pos - offset);
        telemetry.addData("Servo Position", test_pos);
        telemetry.update();
    }

}
