package org.firstinspires.ftc.teamcode.pedroPathing.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp(name="Shooter Test", group="ABC Opmode")

public class ShooterTest extends DecodeLibrary{
    public Servo flap;
    public DcMotorEx shoot1;
    public DcMotorEx shoot2;
    public double test_pos = .04;
    public boolean uppress = false;
    public boolean downpress = false;
    public double speed = 0;
    @Override
    public void init(){
        initialize();
        flap = hardwareMap.get(Servo.class, "flap");
        shoot1 = hardwareMap.get(DcMotorEx.class, "shoot1");
        shoot2 = hardwareMap.get(DcMotorEx.class, "shoot2");
        shoot1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shoot2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flap.setPosition(0);
    }
    @Override
    public void loop(){
        //cameraCode.camera_calculations();
        if(gamepad1.a){
            speed = 1;
        }else if(gamepad1.y){
            speed = 0;
        }
        if(gamepad1.dpad_up){
            uppress = true;
        }else if(!gamepad1.dpad_up && uppress){
            if(gamepad1.right_bumper){
                speed += .05;
            }else {
                test_pos += .01;
            }
            uppress = false;
        } else if(gamepad1.dpad_down){
            downpress = true;
        }else if(!gamepad1.dpad_down && downpress) {
            if(gamepad1.right_bumper){
                speed -= .05;
            }else {
                test_pos -= .01;
            }
            downpress = false;
        }
        flap.setPosition(test_pos);
        shoot1.setVelocity(sppeed);
        shoot2.setVelocity(sppeed);
        telemetry.addData("test pose", test_pos);
        telemetry.addData("test speed", sppeed);
        telemetry.addData("Motor Velocity", shoot1.getVelocity());
        telemetry.update();
    }

}
