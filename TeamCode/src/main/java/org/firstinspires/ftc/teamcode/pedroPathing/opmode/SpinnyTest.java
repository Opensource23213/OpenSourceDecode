package org.firstinspires.ftc.teamcode.pedroPathing.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Disabled
@Config
@TeleOp(name="SpinnyTest", group="ABC Opmode")
public class SpinnyTest extends DecodeLibrary {
    public static double p = 0.0003, i = 0, d = 0;

    public static double f = 0.01;

    @Override
    public void init(){
        spinny.p = p;
        spinny.i = i;
        spinny.d = d;
        spinny.f = f;
        drive_init();
        initialize();
        telemetry.setMsTransmissionInterval(11);
    }
    @Override
    public void loop() {
        spinny.p = p;
        spinny.i = i;
        spinny.d = d;
        spinny.f = f;
        drive();
        button1.button();
        cameraCode.camera_calculations();
        turret.turret_move();
        sensors.sense();
        spinny.spin();
        shooter.shooting();
        flippy.setPosition(flippy_pos);
        telemetry.addData("target", spinny.target);
        telemetry.addData("position", spinny.spin_pos);
        telemetry.addData("turns", spinny.turns);
        telemetry.addData("speed", spinny.speed);
        telemetry.update();
    }



}



