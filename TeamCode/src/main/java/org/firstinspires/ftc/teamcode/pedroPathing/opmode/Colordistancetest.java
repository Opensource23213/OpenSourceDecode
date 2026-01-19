package org.firstinspires.ftc.teamcode.pedroPathing.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DigitalChannel;

@Config
@Disabled
@TeleOp(name="colortest", group="ABC Opmode")

public class Colordistancetest extends DecodeLibrary{
    public AnalogInput apin0;
    public AnalogInput apin1;
    public DigitalChannel sensor;
    public static double red = 0;
    public static double green = 0;
    public static double blue = 0;
    public static double alpha = 0;
    public static double distance = 0;
    @Override
    public void init(){
        sensor = hardwareMap.get(DigitalChannel.class, "apin0");
// set the clock speed on this I2C bus to 400kHz:
        telemetry.setMsTransmissionInterval(11);
    }
    @Override
    public void loop(){
        telemetry.addData("g: ", sensor.getState());

        telemetry.update();
    }
}
