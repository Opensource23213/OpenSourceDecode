package org.firstinspires.ftc.teamcode.pedroPathing.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;

@Config

@TeleOp(name="colortest", group="ABC Opmode")

public class Colordistancetest extends DecodeLibrary{
    public DigitalChannel pin0;
    public DigitalChannel pin1;
    public SRSHub hub;
    @Override
    public void init(){
        hub = hardwareMap.get(SRSHub.class, "hub");
        SRSHub.Config config = new SRSHub.Config();

        config.setEncoder(
                1,
                SRSHub.Encoder.PWM
        );

        config.setEncoder(
                2,
                SRSHub.Encoder.QUADRATURE
        );

        config.setAnalogDigitalDevice(
                1,
                SRSHub.AnalogDigitalDevice.ANALOG
        );
        config.setAnalogDigitalDevice(
                2,
                SRSHub.AnalogDigitalDevice.ANALOG
        );
        hub.init(config);
    }
    @Override
    public void loop(){
        hub.update();
        double pin1 = hub.readAnalogDigitalDevice(1);
        double pin2 = hub.readAnalogDigitalDevice(2);
        telemetry.addData("analog 0", pin1);
        telemetry.addData("analog 1", pin2);
        telemetry.update();
    }
}
