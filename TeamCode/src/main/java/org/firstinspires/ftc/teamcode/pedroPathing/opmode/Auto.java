package org.firstinspires.ftc.teamcode.pedroPathing.opmode;

import static java.lang.Math.abs;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Auto", group="ABC Opmode")
public class Auto extends DecodeLibrary{
    public double forward = 0;
    public double count = 0;
    public ElapsedTime shooot;
    public ElapsedTime delay;
    @Override
    public void init(){
        initialize();
        delay = new ElapsedTime();
        shooot = new ElapsedTime();
        follower.setPose(new Pose());
        cameraCode.limelight.pipelineSwitch(6);
        cameraCode.limelight.start();
        spinny.position = 4;
        flippy_pos = flippy_down;
    }
    @Override
    public void init_loop(){
        spinny.spin();
        delay.reset();
        if(forward == 0) {
            int pos = (int) (spinny.spin_pos - 400) / 1200;
            if(pos < 1){
                pos += 3;
            }
            spinny.position = pos;
            telemetry.addData("pos", pos);
            telemetry.update();
            forward = 1;
        }
    }

    @Override
    public void loop() {
        if (delay.milliseconds() > 5000) {
            follower.update();
            cameraCode.camera_calculations();
            if (forward > 1) {
                turret.turret_move();
            }
            spinny.spin();
            shooter.shooting();
            flippy.setPosition(flippy_pos);

            telemetry.addData("speed shoot", shooter.shoot1.getVelocity());
            telemetry.update();
            if (forward == 1 && cameraCode.result.isValid()) {

                if (cameraCode.result.getFiducialResults().get(0).getFiducialId() == 23) {
                    spinny.position += 1;
                } else if (cameraCode.result.getFiducialResults().get(0).getFiducialId() == 21) {
                    spinny.position -= 1;
                }
                if (spinny.position < 1) {
                    spinny.position = 3;
                } else if (spinny.position > 3) {
                    spinny.position = 1;
                }
                if (cameraCode.result.getTx() < 0) {
                    cameraCode.limelight.pipelineSwitch(5);
                    //follower.setPose(new Pose(0, 0, Math.toRadians(90)));
                    color = 1;
                } else {
                    cameraCode.limelight.pipelineSwitch(4);
                    color = 0;
                }
                forward = 3;
            } else if (forward == 2) {
                shooter.speed = 247.01 * cameraCode.distance_from_target + 1247.05;

                if (shooot.milliseconds() > 10000) {
                    shooot.reset();
                } else {
                    if (shooot.milliseconds() > 5000) {
                        forward = 4;
                    }
                }
                if (color == 1) {
                    shooter.speed = 1800;
                }else{
                    shooter.speed = 1900;
                }
                if (abs(spinny.spin_pos - spinny.target) < 200) {
                    flippy_pos = flippy_up;
                }
                if (abs(spinny.spin_pos - spinny.target) < 200 && abs(turret.turret.getVelocity()) < 20) {
                    spinny.position = 0;
                }
            } else if (forward == 3 && (abs(cameraCode.robot_angle) > 30) && cameraCode.result.isValid()) {
                if (color == 1) {
                    shooter.speed = 1800;
                }else{
                    shooter.speed = 2000;
                }

                angle_offset = turret.turret_angle;
                double pow = 0;
                if ((follower.getPose().getX() > 50 && color == 1) || (follower.getPose().getX() > 62 && color == 0)) {
                    pow = 0;
                    forward = 2;
                    if (color == 1) {
                        follower.setPose(new Pose(0, 0, Math.toRadians(90)));
                    } else {
                        follower.setPose(new Pose(0, 0, Math.toRadians(-90)));
                    }

                } else {
                    pow = .3;
                }
                front_left.setPower(pow);
                front_right.setPower(pow);
                rear_left.setPower(pow);
                rear_right.setPower(pow);

            } else if (forward == 4) {
                double pow = -.3;
                angle_offset = turret.turret_angle;
                front_left.setPower(pow);
                front_right.setPower(pow);
                rear_left.setPower(pow);
                rear_right.setPower(pow);

            }
        }
    }

}
