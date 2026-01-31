package org.firstinspires.ftc.teamcode.pedroPathing.opmode;

import static java.lang.Math.abs;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.SwerveConst;

@Config
@TeleOp
public class ExampleTeleOp extends DecodeLibrary {
    public static Pose startingPose; //See ExampleAuto to understand how to use this
    public static double turn_divider = 2;
    public DcMotor motor;
    public DcMotor motor2;
    public Servo servo1;
    public static double test_pos = .45;
    public static double oscillation = .1;
    public static double oscilation_time = 100;
    public ElapsedTime time = new ElapsedTime();

    @Override
    public void init() {
        x_mod = -3;
        y_mod = 3;
        color = 1;
        follower = SwerveConst.createFollower(hardwareMap, gamepad1);
        follower.setPose(new Pose(0,0, Math.toRadians(-90)));
        shooter.initialize();
        turret.initialize();
        motor = hardwareMap.get(DcMotor.class, "intake");
        motor.setDirection(DcMotorSimple.Direction.REVERSE);
        motor2 = hardwareMap.get(DcMotor.class, "spindexer");
        servo1 = hardwareMap.get(Servo.class, "flippy");
        follower.update();
    }

    @Override
    public void start() {
        //The parameter controls whether the Follower should use break mode on the motors (using it is recommended).
        //In order to use float mode, add .useBrakeModeInTeleOp(true); to your Drivetrain Constants in Constant.java (for Mecanum)
        //If you don't pass anything in, it uses the default (false)
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        //Call this once per loop
        follower.update();
        robot_x = follower.getPose().getX();
        robot_y = follower.getPose().getY();
        robot_heading = follower.getPose().getHeading();
        shooter.speed = shoot_multiplier * ((dead_distance * .0254) - 1.6) + shoot_power_offset;
        shooter.shooting();
        turret.turret_move();
        double mod = (abs(gamepad1.left_stick_y + gamepad1.left_stick_x) * turn_divider);
        if(mod < 1){
            mod = 1;
        }
        double turn = -gamepad1.right_stick_x / mod;

        follower.setTeleOpDrive(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                turn,
                false// Robot Centric
        );
        if(gamepad1.left_trigger > .4){
            motor.setPower(0);
            motor2.setPower(1);
        }else if(gamepad1.right_trigger > .4){
            motor.setPower(-1);
            motor2.setPower(-1);
        }else{
            motor.setPower(0);
            motor2.setPower(0);
        }
        if(time.milliseconds() > oscilation_time * 2){
            servo1.setPosition(test_pos + oscillation);
            time.reset();
        }else if(time.milliseconds() > oscilation_time){
            servo1.setPosition(test_pos);
        }


    }
}