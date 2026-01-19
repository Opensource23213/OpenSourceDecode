package org.firstinspires.ftc.teamcode.pedroPathing.opmode;

import static java.lang.Math.abs;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
@Disabled
@Config
@TeleOp(name="CameraTest", group="ABC Opmode")

public class cameratest extends DecodeLibrary{
    public static double servo_pose = .5;
    public static double power_mod = 35;
    public static double angle_mod = .9;
    public Servo turret_servo_1;
    public Servo turret_servo_2;
    public AnalogInput turret_servo_pos;
    public double turret_pos = 0;
    public double analog_offset = .015;
    public double servo_degrees = 90/.29;
    public double turret_angle = 0;
    public double target_angle = .5;
    public static double max = 15;
    public static double multiplier = 1.2;
    public static double a_slow = 1.2;
    @Override
    public void init(){
        follower = Constants.createFollower(hardwareMap);
        spindexer = hardwareMap.get(DcMotorEx.class, "spindexer");
        flippy = hardwareMap.get(Servo.class, "flippy");
        spindexer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        cameraCode.init();
        drive_init();
        sensors.initialize();
        shooter.initialize();
        turret_servo_1 = hardwareMap.get(Servo.class, "turret_servo_1");
        turret_servo_2 = hardwareMap.get(Servo.class, "turret_servo_2");
        turret_servo_pos = hardwareMap.get(AnalogInput.class, "turret_servo_pos");
    }
    @Override
    public  void loop(){
        drive();
        sensors.sense();
        cameraCode.camera_calculations();
        shooter.shooting();
        turret_pos = (1 - (turret_servo_pos.getVoltage() + analog_offset));
        turret_pos += (turret_pos - .5) * (.03/.26);
        turret_angle = (turret_pos - .5) * servo_degrees;
        cameraCode.camera_calculations();
        double x = 0;
        if(abs(cameraCode.result.getTx()) < max/2){
            x = cameraCode.result.getTx() * a_slow;
        }else{
            if(abs(cameraCode.result.getTx()) < max){
                x = cameraCode.result.getTx() * angle_mod;
            }else{
                x = cameraCode.result.getTx() * multiplier;
            }
        }

        if(cameraCode.result.isValid() && abs(turret_angle) < 90){
            target_angle = (turret_angle + (x * angle_mod)) / servo_degrees + .5;
        }else {
            if(turret_angle > 90){
                target_angle = (90) / servo_degrees + .5;
            }else if(turret_angle < -90){
                target_angle = (-90) / servo_degrees + .5;
            }
        }
        turret_servo_1.setPosition(target_angle);
        turret_servo_2.setPosition(target_angle);
        telemetry.addData("Servo_position", turret_pos);
        telemetry.addData("Turret Angle", turret_angle);
        telemetry.update();
    }
}
