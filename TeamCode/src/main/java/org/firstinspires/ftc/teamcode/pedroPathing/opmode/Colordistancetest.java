package org.firstinspires.ftc.teamcode.pedroPathing.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@Disabled
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
                SRSHub.AnalogDigitalDevice.DIGITAL
        );
        hub.init(config);
        pin0 = hardwareMap.digitalChannel.get("pin0");
        pin1 = hardwareMap.digitalChannel.get("pin1");
    }
    @Override
    public void loop(){
        hub.update();
        double pin = hub.readAnalogDigitalDevice(1);
        telemetry.addData("digital 0", pin0.getState());
        telemetry.addData("digital 0", pin);
        telemetry.addData("digital 1", pin1.getState());
        telemetry.update();
    }
}
