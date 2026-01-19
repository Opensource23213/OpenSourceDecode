package org.firstinspires.ftc.teamcode.pedroPathing.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@Disabled
@TeleOp(name="Shooter Test", group="ABC Opmode")

public class ShooterTest extends DecodeLibrary{
    public Servo flap;
    public PIDController controller;
    public DcMotorEx shoot1;
    public DcMotorEx shoot2;
    public static double test_pos = 0;
    public boolean uppress = false;
    public boolean downpress = false;
    public static double new_adjust = 6000;
    public double speed = 0;
    public static double speed_increase = 0;
    public static double p = 300, i = 1, d = 0;

    public static double f = 8 ;
    public double p_;
    public double i_;
    public double d_;
    public double f_;
    @Override
    public void init(){
        //initialize();
        cameraCode.init();
        sensors.initialize();
        controller = new PIDController(p, i, d);
        flap = hardwareMap.get(Servo.class, "flap");
        shoot1 = hardwareMap.get(DcMotorEx.class, "shoot1");
        shoot2 = hardwareMap.get(DcMotorEx.class, "shoot2");
        PIDFCoefficients pidf = new PIDFCoefficients(p, i, d, f);
        shoot2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
        shoot2.setDirection(DcMotorSimple.Direction.REVERSE);
        shoot2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shoot1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
        shoot1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flippy = hardwareMap.get(Servo.class, "flippy");
        spindexer = hardwareMap.get(DcMotorEx.class, "spindexer");
        spindexer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        flap.setPosition(0);
    }
    @Override
    public void loop(){
        //cameraCode.camera_calculations();
        cameraCode.camera_calculations();
        sensors.sense();
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
        //PIDFCoefficients pidf = new PIDFCoefficients(p + p_, i + i_, d + d_, f + f_);
        //shoot2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
        //double pos = test_pos - ((abs(shoot2.getVelocity() - sppeed)) / new_adjust);
        flap.setPosition(test_pos);
        shoot2.setVelocity(sppeed);
        shoot1.setVelocity(sppeed);
        flippy.setPosition(flippy_pos);
        telemetry.addData("test pose", test_pos);
        telemetry.addData("test speed", sppeed);
        telemetry.addData("Motor Velocity", shoot2.getVelocity());
        telemetry.addData("Motor Velocity", shoot2.getPower());
        telemetry.addData("Distance", cameraCode.distance_from_target);
        telemetry.update();
    }

}
