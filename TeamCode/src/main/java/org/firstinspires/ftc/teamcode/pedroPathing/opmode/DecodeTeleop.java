package org.firstinspires.ftc.teamcode.pedroPathing.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp(name="DecodeTeleop", group="ABC Opmode")
public class DecodeTeleop extends DecodeLibrary {

    @Override
    public void init(){
        initialize();
    }

    @Override
    public void loop(){
        drive();
        button1.button();
        cameraCode.camera_calculations();
        turret.turret_move();
        spinny.spin();
        shooter.shooting();
        flippy.setPosition(flippy_pos);
        telemetry.addData("rpms", shooter.rpms);
        telemetry.update();
    }
}
