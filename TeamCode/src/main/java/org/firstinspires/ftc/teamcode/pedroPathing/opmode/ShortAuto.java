package org.firstinspires.ftc.teamcode.pedroPathing.opmode;

import static java.lang.Math.abs;

import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="ShortAuto", group="ABC Opmode")
public class ShortAuto extends DecodeLibrary{
    public double forward = 0;
    public double count = 0;
    private Path backup;
    @Override
    public void init(){
        initialize();
        cameraCode.limelight.pipelineSwitch(6);
        cameraCode.limelight.start();
        spinny.position = 4;
        flippy_pos = flippy_down;
    }
    @Override
    public void init_loop(){
        spinny.spin();
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
        follower.update();
        cameraCode.camera_calculations();
        if(forward == 2){
            turret.turret_move();
        }
        spinny.spin();
        //shooter.shooting();
        flippy.setPosition(flippy_pos);

        telemetry.addData("speed shoot", shooter.shoot1.getVelocity());
        telemetry.update();
        if(forward == 1 && cameraCode.result.isValid()){
            //shooter.speed = 2300;

            if(cameraCode.result.getFiducialResults().get(0).getFiducialId() == 23){
                spinny.position += 1;
            }else if(cameraCode.result.getFiducialResults().get(0).getFiducialId() == 21){
                spinny.position -= 1;
            }if(spinny.position < 1){
                spinny.position = 3;
            }else if(spinny.position > 3){
                spinny.position = 1;
            }
            if(cameraCode.result.getTx() < 0){
                cameraCode.limelight.pipelineSwitch(5);
                follower.setPose(new Pose(0, 0, Math.toRadians(90)));
                color = 1;
            }else{
                cameraCode.limelight.pipelineSwitch(4);
                color = 0;
            }
            forward = 3;
        }else if(forward == 2){

            if(abs(spinny.spin_pos - spinny.target) < 200){
                flippy_pos = flippy_up;
            }
            if(spinny.speed < 20 && abs(spinny.spin_pos - spinny.target) > 200){
                spinny.p = 10;
            }else{
                spinny.p = .0003;
            }
            if(abs(spinny.spin_pos - spinny.target) < 200 && shooter.shoot1.getVelocity() > 2280 && abs(turret.turret.getVelocity()) < 20){
                spinny.position -= 1;
                count += 1;
            }
            if(spinny.position < 1){
                spinny.position = 3;
            }
            if(count > 5){
                forward = 3;
            }
        }else if(forward == 3){
            angle_offset = turret.turret_angle;
            double pow = .2;
            front_left.setPower(pow);
            front_right.setPower(pow);
            rear_left.setPower(pow);
            rear_right.setPower(pow);
        }
    }

}
