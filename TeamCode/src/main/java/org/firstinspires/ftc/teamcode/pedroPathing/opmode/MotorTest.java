package org.firstinspires.ftc.teamcode.pedroPathing.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp(name="Motor Test", group="ABC Opmode")

public class MotorTest extends OpMode {
    public DcMotor motor;
    public DcMotor motor2;
    public Servo servo1;
    public static double test_pos = 0;
    @Override
    public void init(){
        motor = hardwareMap.get(DcMotor.class, "intake");
        motor2 = hardwareMap.get(DcMotor.class, "spindexer");
        servo1 = hardwareMap.get(Servo.class, "flippy");
    }
    @Override
    public void loop(){
        motor.setPower(gamepad1.left_stick_y);
        motor2.setPower(gamepad1.right_trigger - gamepad1.left_trigger);
        if(gamepad1.right_trigger > .4){
            test_pos = .45;
        }else{
            test_pos = .68;
        }
        servo1.setPosition(test_pos);
        telemetry.addData("Servo Position", test_pos);
        telemetry.update();
    }

}
