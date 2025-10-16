package org.firstinspires.ftc.teamcode.pedroPathing.opmode;

import static java.lang.Math.abs;

import android.widget.Spinner;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


@Config
@Disabled
@TeleOp(name="SpinnyTest", group="ABC Opmode")
public class SpinnyTest extends DecodeLibrary {
    public static double p = 0.0002, i = 0, d = 0;

    public static double f = 0.01;

    @Override
    public void init(){
        spinny.p = p;
        spinny.i = i;
        spinny.d = d;
        spinny.f = f;
        initialize();
        telemetry.setMsTransmissionInterval(4);
    }
    @Override
    public void loop() {
        if(gamepad1.a){
            spinny.position = 1;
        }else if(gamepad1.b){
            spinny.position = 2;
        }else if(gamepad1.dpad_up){
            spinny.position = 3;
        }else{
            spinny.position = 0;
        }
        spinny.spin();
        telemetry.addData("target", spinny.target);
        telemetry.addData("position", spinny.spin_pos);
        telemetry.addData("turns", spinny.turns);
        telemetry.addData("speed", spinny.speed);
        telemetry.update();
    }



}



