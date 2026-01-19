package org.firstinspires.ftc.teamcode.pedroPathing.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

//@Disabled
@Config
@Disabled
@TeleOp(name="NewSpinnyTest", group="ABC Opmode")
public class NewSpinnyTest extends DecodeLibrary {
    
    public static double flippy_offset = -.05;
    public static double shoot_increase = 1700;
    public boolean buttonpress = false;
    @Override
    public void init(){
        follower = Constants.createFollower(hardwareMap);
        drive_init();
        sensors.initialize();
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        shooter.initialize();
        cameraCode.init();
        turret.initialize();
        spindexer = hardwareMap.get(DcMotorEx.class, "spindexer");
        flippy = hardwareMap.get(Servo.class, "flippy");
        spindexer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        balls.clear();
    }
    @Override
    public void loop() {
        /*drive();
        sensors.sense();
        cameraCode.camera_calculations();
        turret.turret_move();
        shooter.shooting();
        flippy.setPosition(flippy_pos);
        if(gamepad1.right_bumper){
            buttonpress = true;
        }else if(!gamepad1.right_bumper && buttonpress){
            buttonpress = false;
            if(shooter.speed == 0){
                shooter.speed = shoot_multiplier * (cameraCode.distance_from_target) + shoot_power_offset;
            }else{
                shooter.speed = 0;
            }

        }
        if(shooter.speed != 0){
            shooter.speed = shoot_multiplier * (cameraCode.distance_from_target) + shoot_power_offset;
        }
        telemetry.addData("balls counted", balls.size());*/
        spindexer.setPower(gamepad1.right_trigger - gamepad1.left_trigger);


    }



}



