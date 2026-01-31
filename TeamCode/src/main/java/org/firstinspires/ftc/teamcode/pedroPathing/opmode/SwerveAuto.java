package org.firstinspires.ftc.teamcode.pedroPathing.opmode;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.SwerveConst;

@Autonomous(name="SwerveAuto", group="ABC Opmode")

public class SwerveAuto extends DecodeLibrary {
    public Path first_pick;
    public Path pick_after_stuff;
    public Path first_shoot;
    public Path second_shoot;
    public Path third_shoot;
    public Path fourth_shoot;
    public Path fifth_shoot;
    public Path second_pick;
    public Path third_pick;
    public Path park;
    public Path gate;
    public static double fr_offset = 1050;
    public static double fl_offset = -1350;
    public static double rr_offset = -900;
    public static double rl_offset = -160;
    public double steps = 0;
    public boolean shooting = false;

    public static double p = .001, i = 0, d = .00005;

    public static double f = 0;

    public boolean move = false;
    public double forward = 0;
    public Path path2;
    public ElapsedTime timer = null;
    @Override
    public void init(){
        x_mod = 2;
        y_mod = -12;
        color = 0;
        timer = new ElapsedTime();
        follower = SwerveConst.createFollower(hardwareMap, gamepad1);
        follower.setPose(new Pose(0,-.375,Math.toRadians(90)));
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        spindexer = hardwareMap.get(DcMotorEx.class, "spindexer");
        flippy = hardwareMap.get(Servo.class, "flippy");
        spindexer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter.initialize();
        turret.initialize();
        blue_init();
        follower.followPath(first_shoot);
    }
    @Override
    public void init_loop(){
        follower.drivetrain.setYVelocity(0);
        follower.update();
    }
    @Override
    public void loop() {
        shoot();
        follower.update();
        robot_x = follower.getPose().getX();
        robot_y = follower.getPose().getY();
        robot_heading = follower.getPose().getHeading();
        intake.setPower(-1);
        flippy.setPosition(.45);
        shooter.shooting();
        turret.turret_move();
        if(follower.atParametricEnd() || !follower.isBusy() || follower.drivetrain.yVelocity() == 1 || steps == 2) {
            if (forward == 0) {
                if (steps == 0) {
                    follower.followPath(third_pick);
                    shooting = true;
                }
                if (steps == 2) {
                    forward = 1;
                    shooting = false;
                    steps = 0;
                }
            } else if (forward == 1) {
                follower.followPath(fourth_shoot);
                forward = 2;
            } else if (forward == 2) {
                if (steps == 0) {
                    follower.followPath(first_pick);
                    shooting = true;
                }
                if (steps == 2) {
                    forward = 3;
                    shooting = false;
                    steps = 0;
                }
            } else if (forward == 3) {
                follower.followPath(gate);
                forward = 4;
            } else if (forward == 4) {
                follower.followPath(second_shoot);
                forward = 5;
            } else if (forward == 5) {
                if (steps == 0) {
                    follower.followPath(second_pick);
                    shooting = true;
                }
                if (steps == 2) {
                    forward = 6;
                    shooting = false;
                    steps = 0;
                }
            } else if (forward == 6) {
                follower.followPath(third_shoot);
                forward = 7;
            } else if (forward == 7) {
                if (steps == 0) {
                    follower.followPath(pick_after_stuff);
                    shooting = true;
                }
                if (steps == 2) {
                    forward = 8;
                    shooting = false;
                    steps = 0;
                }
            } else if (forward == 8) {
                follower.followPath(fifth_shoot);
                forward = 9;
            }

        }
    }
    public void blue_init(){
        first_shoot = new Path(new BezierCurve(new Pose(0,-.375), new Pose(72, 4)));
        first_shoot.setConstantHeadingInterpolation(Math.toRadians(90));
        first_pick = new Path(new BezierCurve(new Pose(65,0), new Pose(53, 3), new Pose(51, 37)));
        first_pick.setConstantHeadingInterpolation(Math.toRadians(90));
        gate = new Path(new BezierCurve(first_pick.getLastControlPoint(), new Pose(54, 41)));
        gate.setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180));
        second_shoot = new Path(new BezierCurve(gate.getLastControlPoint(), new Pose(56, 15), new Pose(72, 4)));
        second_shoot.setLinearHeadingInterpolation(Math.toRadians(180),Math.toRadians(45));
        second_pick = new Path(new BezierLine(second_shoot.getLastControlPoint(), new Pose(79, 33.5)));
        second_pick.setConstantHeadingInterpolation(Math.toRadians(90));
        third_shoot = new Path(new BezierLine(second_pick.getLastControlPoint(), new Pose(72, 4)));
        third_shoot.setConstantHeadingInterpolation(Math.toRadians(90));
        third_pick = new Path(new BezierCurve(third_shoot.getLastControlPoint(), new Pose(32, 0), new Pose(30, 38)));
        third_pick.setConstantHeadingInterpolation(Math.toRadians(90));
        fourth_shoot = new Path(new BezierCurve(third_pick.getLastControlPoint(), new Pose(35, 0), new Pose(72, 4)));
        fourth_shoot.setConstantHeadingInterpolation(Math.toRadians(90));
        pick_after_stuff = new Path(new BezierCurve(new Pose(72, 4), new Pose(61,45), new Pose(21, 47), new Pose(6, 47)));
        fifth_shoot = new Path(new BezierLine(pick_after_stuff.getLastControlPoint(), new Pose(72, 4)));
        fifth_shoot.setLinearHeadingInterpolation(Math.toRadians(180),Math.toRadians(150));
        park = new Path(new BezierLine(fourth_shoot.getLastControlPoint(), new Pose(72,15)));
        park.setConstantHeadingInterpolation(Math.toRadians(45));
    }
    public void shoot(){
        if(shooting){
            follower.drivetrain.setYVelocity(0);
            if(color == 1){
                shooter.speed = 1710;
            }else {
                shooter.speed = 1710;
            }
            if(steps == 0){
                timer.reset();
                steps = .5;
                /*if(forward == .25){
                    steps = .5;
                }else{
                    spindexer.setPower(1);
                    flippy_pos = flippy_up;
                }*/
            }else if(steps == .5 && timer.milliseconds() > 150){
                timer.reset();
                steps = 1;
                spindexer.setPower(1);
            }else if(steps == 1 && timer.milliseconds() > 1220){
                steps = 2;
                shooting = false;
            }
        }else{
            if(color == 1){
                shooter.speed = 1710;
            }else {
                shooter.speed = 1710;
            }
            spindexer.setPower(-1);
        }
    }

}
