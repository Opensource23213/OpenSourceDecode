package org.firstinspires.ftc.teamcode.pedroPathing.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;


@Config
@Disabled
@TeleOp(name="IntakeTest", group="ABC Opmode")
public class IntakeTest extends DecodeLibrary {
    public double test_speed = 0;
    public boolean uppress = false;
    public boolean downpress = false;

    @Override
    public void init(){
        initialize();
        drive_init();
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
    }
    @Override
    public void loop() {
        if(gamepad1.a){
            test_speed = 1;
        }else if(gamepad1.b){
            test_speed = 0;
        }else if(gamepad1.dpad_up){
            uppress = true;
        }else if(!gamepad1.dpad_up && uppress){
            test_speed += .05;
            uppress = false;
        } else if(gamepad1.dpad_down){
            downpress = true;
        }else if(!gamepad1.dpad_down && downpress) {
            test_speed -= .05;
            downpress = false;
        }
        intake.setPower(test_speed);
        rear_right.setPower(test_speed);
        telemetry.addData("Speed", test_speed);
        telemetry.addData("Current", intake.getCurrent(CurrentUnit.AMPS));
        telemetry.update();
    }



}



