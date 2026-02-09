package org.firstinspires.ftc.teamcode.pedroPathing.opmode;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name="Robosapiens Auto", group="ABC Opmode", preselectTeleOp = "DecodeTeleop")
public class Robosapiens_Auto extends DecodeLibrary{
    public double forward = 0.25;
    public Path first_pick;
    public Path pick_after_stuff;
    public Path pick_after_stuff2;
    public Path first_shoot;
    public Path second_shoot;
    public Path third_shoot;
    public Path fourth_shoot;
    public Path fifth_shoot;
    public Path fifth_shoot2;
    public Path second_pick;
    public Path third_pick;
    public Path park;
    public Path gate;
    public Path gate2;
    public double shot = 0;
    public double old_turns = 0;
    public double steps = 0;
    public ElapsedTime gate_open = new ElapsedTime();
    public ElapsedTime shooting_time = new ElapsedTime();
    public sorting sorting = new sorting();
    public double sort_pos = 0;
    public double old_pattern = 2;
    public double old_color = 1;
    public boolean shooting = false;
    public double index_steps = 0;
    public boolean re_init = false;
    @Override
    public void init(){
        color = 1;
        teleop = false;
        initialize();
        drive_init_auto();
        cameraCode.limelight.pipelineSwitch(6);
        if(color == 0) {
            follower.setPose(new Pose(122,25));
            blue_init();
            angle_offset = -90;
            old_color = 0;
        }else{
            follower.setPose(new Pose(122,-25));
            red_init();
            angle_offset = 90;
            old_color = 1;
        }
        tele_offset = turret.current_angle;
        pattern = 2;
        follower.setMaxPower(1);
        sensors.sorted = true;
    }
    @Override
    public void init_loop(){
        follower.update();
        cameraCode.camera_calculations();
        follower.setMaxPowerScaling(.7);
        turret.turret_move();
        if (cameraCode.limelight.getLatestResult().getPipelineIndex() != 6) {
            cameraCode.limelight.pipelineSwitch(6);
        }
        turret.zero = true;
        if (color == 0) {
            turret.manual_angle = 54;
        } else {
            turret.manual_angle = -50;
        }
        if(gamepad1.a){
            color = 1;
            re_init = true;
        }else if(gamepad1.b){
            color = 0;
            re_init = true;
        }

        if (cameraCode.result.isValid()) {
            if (color == 0) {
                if (cameraCode.result.getFiducialResults().get(0).getFiducialId() == 21) {
                    pattern = 3;
                } else if (cameraCode.result.getFiducialResults().get(0).getFiducialId() == 22) {
                    pattern = 1;
                } else {
                    pattern = 2;
                }
            } else {
                if (cameraCode.result.getFiducialResults().get(0).getFiducialId() == 22) {
                    pattern = 3;
                } else if (cameraCode.result.getFiducialResults().get(0).getFiducialId() == 23) {
                    pattern = 1;
                } else {
                    pattern = 2;
                }
            }
        }
        if(old_color != color || re_init) {
            re_init = false;
            initialize();
            cameraCode.limelight.pipelineSwitch(6);
            if (color == 0) {
                follower.setPose(new Pose(122,25));
                blue_init();
                angle_offset = -90;
                old_color = 0;
            } else {
                follower.setPose(new Pose(122,-25));
                red_init();
                angle_offset = 90;
                old_color = 1;
            }
        }
        auto_pose = follower.getPose();
        telemetry.addData("color", color);
        telemetry.addData("pattern", pattern);
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.update();
    }

    @Override
    public void loop() {
        follower.update();
        shooter.position = 0;
        auto_pose = follower.getPose();
        cameraCode.camera_calculations();
        if(cameraCode.result.getPipelineIndex() == 6){
            follower.followPath(first_shoot);
            if (color == 1) {
                cameraCode.limelight.pipelineSwitch(5);
            } else {
                cameraCode.limelight.pipelineSwitch(4);
            }
        }
        flippy.setPosition(flippy_pos);
        sensors.auto_index();
        shooter.shooting();

        if(forward == 5){
            if(follower.getPose().getX() < 45){
                follower.setMaxPower(.45);
            }
        }

        turret.turret_move();
        sorting.sort();
        shoot();
        if(forward == .25 && color == 1){
            y_mod = 8;
        }else if(color == 1){
            y_mod = 3;
        }
        if(follower.atParametricEnd() || !follower.isBusy()) {
            if(forward == .25){
                shoot();
                shooting = true;
                intake.setPower(0);
                if(steps == 2) {
                    shooting = false;
                    flippy_pos = flippy_hold;
                    spindexer.setPower(-1);
                    intake.setPower(1);
                    follower.followPath(second_pick);
                    follower.setMaxPower(.9);
                    forward = .5;
                    steps = 0;
                }
            }else if(forward == .5){
                sensors.sorted = true;
                follower.followPath(third_shoot);
                follower.setMaxPower(1);
                forward = .75;
            }else if (forward == .75) {
                shoot();
                shooting = true;
                intake.setPower(0);
                if(steps == 2){
                    shooting = false;
                    flippy_pos = flippy_hold;
                    robot_going_forward = true;
                    spindexer.setPower(-1);
                    follower.followPath(first_pick);
                    intake.setPower(1);
                    follower.setMaxPower(1);
                    forward = .8;
                    steps = 0;
                }
            }else if (forward == .8) {
                intake.setPower(1);
                follower.setMaxPowerScaling(1);
                follower.followPath(gate);
                follower.setMaxPower(1);
                forward = 1;
            }else if(forward == 1){
                if(steps == 0){
                    gate_open.reset();
                    intake.setPower(-1);
                    steps = 1;
                }else if(steps == 1 && gate_open.milliseconds() > 1400) {
                    robot_going_forward = true;
                    sensors.sorted = true;
                    follower.followPath(second_shoot);
                    follower.setMaxPower(1);
                    forward = 2;
                    steps = 0;
                }
            }else if(forward == 2){
                intake.setPower(0);
                shoot();
                shooting = true;
                if(steps == 2) {
                    shooting = false;
                    flippy_pos = flippy_hold;
                    spindexer.setPower(0);
                    intake.setPower(0);
                    follower.followPath(gate2);
                    follower.setMaxPower(1);
                    forward = 3;
                    steps = 0;
                }
            }else if(forward == 3){
                if(steps == 0){
                    gate_open.reset();
                    steps = 1;
                }else if(steps == 1 && gate_open.milliseconds() > 1500) {
                    follower.followPath(pick_after_stuff);
                    intake.setPower(1);
                    spindexer.setPower(-1);
                    follower.setMaxPower(1);
                    forward = 5;
                    steps = 0;
                }
            }else if(forward == 5){
                follower.setMaxPowerScaling(1);
                index_reverse = true;
                sorting.sort_balls = true;
                follower.followPath(fifth_shoot2);
                follower.setMaxPower(1);
                forward = 8.1;
            }
            else if(forward == 8.1){
                shoot();
                shooting = true;
                if(steps == 2) {
                    shooting = false;
                    flippy_pos = flippy_hold;
                    spindexer.setPower(0);
                    intake.setPower(0);
                    shooter.speed = 0;
                    follower.setMaxPower(1);
                    forward = 9;
                    steps = 0;
                }
            }
        }
    }

    public void red_init(){
        first_shoot = new Path(new BezierLine(new Pose(123,-24), new Pose(72, -4)));
        first_shoot.setLinearHeadingInterpolation(0,Math.toRadians(-45));
        first_pick = new Path(new BezierCurve(new Pose(65,0), new Pose(50, -3), new Pose(48, -37)));
        first_pick.setConstantHeadingInterpolation(Math.toRadians(-90));
        gate = new Path(new BezierCurve(first_pick.getLastControlPoint(), new Pose(54, -41)));
        gate.setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(-180));
        second_shoot = new Path(new BezierCurve(gate.getLastControlPoint(), new Pose(56, -15), new Pose(72, -4)));
        second_shoot.setLinearHeadingInterpolation(Math.toRadians(-180),Math.toRadians(-45));
        second_pick = new Path(new BezierLine(second_shoot.getLastControlPoint(), new Pose(76, -35)));
        second_pick.setConstantHeadingInterpolation(Math.toRadians(-90));
        third_shoot = new Path(new BezierLine(second_pick.getLastControlPoint(), new Pose(72, -4)));
        third_shoot.setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(-45));
        third_pick = new Path(new BezierCurve(third_shoot.getLastControlPoint(), new Pose(17, -5), new Pose(26, -38)));
        third_pick.setConstantHeadingInterpolation(Math.toRadians(-90));
        fourth_shoot = new Path(new BezierCurve(third_pick.getLastControlPoint(), new Pose(35, -5), new Pose(72, -4)));
        fourth_shoot.setLinearHeadingInterpolation(Math.toRadians(-90),Math.toRadians(-45));
        gate2 = new Path(new BezierCurve(second_shoot.getLastControlPoint(), new Pose(54, -40)));
        gate2.setLinearHeadingInterpolation(Math.toRadians(-45), Math.toRadians(-180));
        pick_after_stuff = new Path(new BezierCurve(gate2.getLastControlPoint(), new Pose(49,-41), new Pose(21, -43), new Pose(2, -43)));
        pick_after_stuff.setConstantHeadingInterpolation(Math.toRadians(-170));
        fifth_shoot = new Path(new BezierLine(pick_after_stuff.getLastControlPoint(), new Pose(72, -4)));
        fifth_shoot.setLinearHeadingInterpolation(Math.toRadians(-180),Math.toRadians(-110));
        pick_after_stuff2 = new Path(new BezierCurve(new Pose(72, -4), new Pose(61,-43), new Pose(21, -43), new Pose(2, -43)));
        pick_after_stuff2.setConstantHeadingInterpolation(Math.toRadians(-170));
        fifth_shoot2 = new Path(new BezierLine(pick_after_stuff.getLastControlPoint(), new Pose(91, 0)));
        fifth_shoot2.setLinearHeadingInterpolation(Math.toRadians(-180),Math.toRadians(-110));
        park = new Path(new BezierLine(fourth_shoot.getLastControlPoint(), new Pose(90,-2)));
        park.setConstantHeadingInterpolation(Math.toRadians(-110));
    }
    public void blue_init(){
        first_shoot = new Path(new BezierLine(new Pose(123,24), new Pose(72, 4)));
        first_shoot.setLinearHeadingInterpolation(0,Math.toRadians(45));
        first_pick = new Path(new BezierCurve(new Pose(65,0), new Pose(50, 3), new Pose(48, 37)));
        first_pick.setConstantHeadingInterpolation(Math.toRadians(90));
        gate = new Path(new BezierCurve(first_pick.getLastControlPoint(), new Pose(54, 41)));
        gate.setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180));
        second_shoot = new Path(new BezierCurve(gate.getLastControlPoint(), new Pose(56, 15), new Pose(72, 4)));
        second_shoot.setLinearHeadingInterpolation(Math.toRadians(180),Math.toRadians(45));
        second_pick = new Path(new BezierLine(second_shoot.getLastControlPoint(), new Pose(76, 35)));
        second_pick.setConstantHeadingInterpolation(Math.toRadians(90));
        third_shoot = new Path(new BezierLine(second_pick.getLastControlPoint(), new Pose(72, 4)));
        third_shoot.setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(45));
        third_pick = new Path(new BezierCurve(third_shoot.getLastControlPoint(), new Pose(17, 5), new Pose(26, 38)));
        third_pick.setConstantHeadingInterpolation(Math.toRadians(90));
        fourth_shoot = new Path(new BezierCurve(third_pick.getLastControlPoint(), new Pose(35, 5), new Pose(72, 4)));
        fourth_shoot.setLinearHeadingInterpolation(Math.toRadians(90),Math.toRadians(45));
        gate2 = new Path(new BezierCurve(second_shoot.getLastControlPoint(), new Pose(57, 40)));
        gate2.setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(180));
        pick_after_stuff = new Path(new BezierCurve(gate2.getLastControlPoint(), new Pose(49,41), new Pose(21, 50), new Pose(2, 50)));
        pick_after_stuff.setConstantHeadingInterpolation(Math.toRadians(170));
        fifth_shoot = new Path(new BezierLine(pick_after_stuff.getLastControlPoint(), new Pose(72, 4)));
        fifth_shoot.setLinearHeadingInterpolation(Math.toRadians(180),Math.toRadians(110));
        pick_after_stuff2 = new Path(new BezierCurve(new Pose(72, 4), new Pose(61,43), new Pose(21, 43), new Pose(2, 43)));
        pick_after_stuff2.setConstantHeadingInterpolation(Math.toRadians(170));
        fifth_shoot2 = new Path(new BezierLine(pick_after_stuff.getLastControlPoint(), new Pose(91, 0)));
        fifth_shoot2.setLinearHeadingInterpolation(Math.toRadians(180),Math.toRadians(110));
        park = new Path(new BezierLine(fourth_shoot.getLastControlPoint(), new Pose(90,2)));
        park.setConstantHeadingInterpolation(Math.toRadians(110));
    }
    public void shoot(){
        if(shooting){
            intake.setPower(-1);
            if(forward == 8.1){
                if (color == 1) {
                    shooter.speed = 1640;
                } else {
                    shooter.speed = 1640;
                }
            }else {
                if (color == 1) {
                    shooter.speed = 1710;
                } else {
                    shooter.speed = 1710;
                }
            }
            if(steps == 0 && sensors.sorted){
                shooting_time.reset();
                steps = .5;
                /*if(forward == .25){
                    steps = .5;
                }else{
                    spindexer.setPower(1);
                    flippy_pos = flippy_up;
                }*/
            }else if(steps == .5 && shooting_time.milliseconds() > 200){
                shooting_time.reset();
                steps = 1;
                spindexer.setPower(.85);
                flippy_pos = flippy_up;
            }else if(steps == 1 && shooting_time.milliseconds() > 1420){
                if(forward != 8.1) {
                    steps = 2;
                    sensors.sorted = false;
                }
            }
        }else{
            if(color == 1){
                shooter.speed = 1710;
            }else {
                shooter.speed = 1710;
            }
        }
    }
    public class sorting{
        public boolean sort_balls = false;
        public void sort(){
            if(sort_balls){
                sensors.sort();
                if(sensors.sorted){
                    sort_balls = false;
                }
            }
        }
    }


}
