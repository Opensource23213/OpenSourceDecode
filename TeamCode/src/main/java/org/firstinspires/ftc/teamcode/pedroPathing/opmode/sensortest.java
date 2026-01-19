package org.firstinspires.ftc.teamcode.pedroPathing.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.lynx.LynxI2cDeviceSynch;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchSimple;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
@Disabled
@Config
@TeleOp(name="sensorcalibrate", group="Calibrate")
public class sensortest extends LinearOpMode {
    public static int range = 300;
    @Override

    public void runOpMode() throws InterruptedException {
        /*ColorRangefinder crf = new ColorRangefinder(hardwareMap.get(RevColorSensorV3.class, "colorfront1"));
        waitForStart();
        crf.setI2cAddress(0x52);
        crf.getCalibration();
        crf.readDistance();
        crf.setLedBrightness(999);

        crf.setPin0Digital(ColorRangefinder.DigitalMode.HSV, 160 / 360.0 * 255, 190 / 360.0 * 255); // purple
        crf.setPin0DigitalMaxDistance(ColorRangefinder.DigitalMode.HSV, 30); // 10mm or closer requirement
        crf.setPin1DigitalMaxDistance(ColorRangefinder.DigitalMode.DISTANCE, 100);
        crf.setPin1Digital(ColorRangefinder.DigitalMode.DISTANCE, 0, 30); // green
           */

        LaserRangefinder lrf = new LaserRangefinder(hardwareMap.get(RevColorSensorV3.class, "colorfront1"));
        lrf.setI2CAddress(0x52);
        waitForStart();
        lrf.getROI();
        lrf.getPin0Mode();
        lrf.setDistanceMode(LaserRangefinder.DistanceMode.SHORT);
        lrf.setTiming(10,0);
        lrf.setPin0Digital(0, 75);
        lrf.setPin1Digital(0, range);
        lrf.getScanDistance(DistanceUnit.MM);
        telemetry.addData("", lrf.getStatus());
        telemetry.update();
        sleep(1000);
        lrf.getDistance(DistanceUnit.MM);

    }
}
/**
 * Helper class for configuring the Brushland Labs Laser Rangefinder.
 * Online documentation: <a href="https://docs.brushlandlabs.com">...</a>
 */
class LaserRangefinder {
    public final LynxI2cDeviceSynch i2c;

    public LaserRangefinder(RevColorSensorV3 device) {
        this.i2c = (LynxI2cDeviceSynch) device.getDeviceClient();
        this.i2c.enableWriteCoalescing(true);
    }

    /**
     * Set pin one to threshold for a range.
     */
    public void setPin0Digital(int thresh_lo, int thresh_hi) {
        setPin(R_PIN0_MODE, M_DIG, thresh_lo, thresh_hi);
    }

    public void setPin1Digital(int thresh_lo, int thresh_hi) {
        setPin(R_PIN1_MODE, M_DIG, thresh_lo, thresh_hi);
    }

    public void setPin0Analog(int bound_lo, int bound_hi) {
        setPin(R_PIN0_MODE, M_ANA, bound_lo, bound_hi);
    }

    public void setPin1Analog(int bound_lo, int bound_hi) {
        setPin(R_PIN1_MODE, M_ANA, bound_lo, bound_hi);
    }

    private void setPin(byte reg, byte reg2, int lo, int hi) {
        byte lo0 = (byte) (lo & 0xFF);
        byte lo1 = (byte) ((lo & 0xFF00) >> 8);
        byte hi0 = (byte) (hi & 0xFF);
        byte hi1 = (byte) ((hi & 0xFF00) >> 8);
        write(reg, new byte[]{reg2, lo0, lo1, hi0, hi1});
    }

    public int getPin0Mode() {
        return i2c.read8(R_PIN0_MODE);
    }

    public int getPin1Mode() {
        return i2c.read8(R_PIN1_MODE);
    }

    public void setDistanceMode(DistanceMode mode) {
        write(R_DISTMODE, new byte[]{(byte) (mode.ordinal() + 1)});
    }

    public DistanceMode getDistanceMode() {
        byte v = i2c.read8(R_DISTMODE);
        switch (v) {
            case 1: return DistanceMode.SHORT;
            case 2: return DistanceMode.MEDIUM;
            case 3: return DistanceMode.LONG;
            default: throw new RuntimeException("Could not get distance mode, " + v);
        }
    }

    /**
     * Set the timing budget and the total measurement period in milliseconds.
     * A period of zero means that the next range will begin immediately after the completion of the current range.
     */
    public void setTiming(int budget, int period) {
        if (budget < 5 || budget > 1000) throw new RuntimeException("Invalid timing budget: " + budget);
        if (period != 0 && (period < budget + 3)) throw new RuntimeException("Measurement period must be at least 4ms more than timing budget if set. For fast ranging, use a period of 0.");
        write(R_TIMING, new byte[]{(byte) (budget & 0xFF), (byte) ((budget & 0xFF00) >> 8), (byte) (period & 0xFF), (byte) ((period & 0xFF00) >> 8)});
    }

    public int[] getTiming() {
        java.nio.ByteBuffer buf = java.nio.ByteBuffer.wrap(i2c.read(R_TIMING, 4)).order(java.nio.ByteOrder.LITTLE_ENDIAN);
        return new int[]{buf.getShort(), buf.getShort()};
    }

    /**
     * Set the size of the sensor's region of interest. The ROI must be at least 4x4 in size.
     */
    public void setROI(int topLeftX, int topLeftY, int botRightX, int botRightY) {
        if (botRightX - topLeftX < 3 || topLeftY - botRightY < 3) throw new RuntimeException("ROI too small, must be at least 4x4.");
        for (int v : new int[]{topLeftX, topLeftY, botRightX, botRightY}) if (v < 0 || v > 15) throw new RuntimeException("Invalid ROI point value: " + v);
        write(R_ROI_TLX, new byte[]{(byte) topLeftX, (byte) topLeftY, (byte) botRightX, (byte) botRightY});
    }

    public int[] getROI() {
        byte[] data = i2c.read(R_ROI_TLX, 4);
        return new int[]{data[0], data[1], data[2], data[3]};
    }

    /**
     * Give the sensor a new I2C address from the default of 0x52.
     */
    public void setI2CAddress(int newAddress) {
        if (newAddress < 1 || newAddress > 127) throw new RuntimeException("Invalid I2C address: " + newAddress);
        write(R_IIC_ADDR, new byte[]{(byte) newAddress});
    }

    /**
     * Returns the (x, y) coordinates of the factory calibrated center of the sensor's 16x16 ROI.
     */
    public int[] getOpticalCenter() {
        byte[] data = i2c.read(R_OPTCENTERX, 2);
        return new int[]{data[0], data[1]};
    }

    /**
     * Resets from I2C scan mode to normal I2C mode, clearing previously configured scan ROIs.
     */
    public void setI2C() { // also clears custom i2c address
        write(R_PIN0_MODE, new byte[]{M_I2C, 0, 0, 0, 0});
    }

    public ScanSequenceBuilder setAnalogScanMode() {
        return new ScanSequenceBuilder(M_AN2);
    }

    public ScanSequenceBuilder setI2CScanMode() {
        return new ScanSequenceBuilder(M_II2);
    }

    public class ScanSequenceBuilder {
        private final byte mode;

        public ScanSequenceBuilder(byte mode) {
            this.mode = mode;
        }

        private final java.util.List<byte[]> rois = new java.util.ArrayList<>();

        public ScanSequenceBuilder addScanROI(
                int topLeftX,
                int topLeftY,
                int botRightX,
                int botRightY
        ) {
            rois.add(new byte[]{(byte) topLeftX, (byte) topLeftY, (byte) botRightX, (byte) botRightY});
            return this;
        }

        public void setScanROIs() {
            for (byte[] roi : rois) i2c.write(R_PIN0_MODE, new byte[]{mode, roi[0], roi[1], roi[2], roi[3]});
            write(R_PIN0_MODE, new byte[]{mode, (byte) 0xFF, (byte) 0xFF, (byte) 0xFF, (byte) 0xFF});
        }
    }

    private int status = 0;

    /**
     * Retrieve the status of the last reading, where 0 indicates the reading is good, 1-2 indicates okay,
     * larger numbers indicates bad readings.
     */
    public int getStatus() {
        return status;
    }

    /**
     * Read distance via I2C. This also populates the value returned from {@link LaserRangefinder#getStatus}
     */
    public double getDistance(DistanceUnit unit) {
        byte[] data = i2c.read(R_PS_DATA_0, 2);
        status = (data[1] & 0xE0) >> 5;
        data[1] &= 0x1F;
        return unit.fromUnit(DistanceUnit.MM, java.nio.ByteBuffer.wrap(data, 0, 2)
                .order(java.nio.ByteOrder.LITTLE_ENDIAN)
                .getShort()
        );
    }

    private int b3 = 0;

    /**
     * In I2C scan mode, this returns the index of the ROI of the last reading from
     * {@link LaserRangefinder#getDistance(DistanceUnit)}.
     */
    public int getROINum() {
        return b3;
    }

    public double getScanDistance(DistanceUnit unit) {
        byte[] data = i2c.read(R_PS_DATA_0, 3);
        b3 = data[2];
        status = data[1] & 0xE0;
        data[1] &= 0x1F;
        return unit.fromUnit(DistanceUnit.MM, java.nio.ByteBuffer.wrap(data, 0, 2)
                .order(java.nio.ByteOrder.LITTLE_ENDIAN)
                .getShort()
        );
    }

    private void write(int creg, byte[] bytes) {
        i2c.write(creg, bytes);
        try {
            Thread.sleep(10);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
    }

    public enum DistanceMode {
        /**
         * Maximum range of 1.3m, good immunity to ambient light.
         */
        SHORT,
        /**
         * Maximum range of 3m, okay immunity to ambient light.
         */
        MEDIUM,
        /**
         * Maximum range of 4m, more susceptible to ambient light.
         */
        LONG
    }

    private static final byte M_I2C = 0;
    private static final byte M_ANA = 1;
    private static final byte M_DIG = 2;
    private static final byte M_AN2 = 3;
    private static final byte M_II2 = 4;

    private static final byte R_PS_DATA_0 = 0x08;
    private static final byte R_PIN0_MODE = 0x28;
    private static final byte R_PIN1_MODE = 0x2D;
    private static final byte R_ROI_TLX = 0x32;
    private static final byte R_DISTMODE = 0x36;
    private static final byte R_TIMING = 0x37; // timing 0
    private static final byte R_IIC_ADDR = 0x3B;
    private static final byte R_OPTCENTERX = 0x3C;
}
/**
 * Helper class for configuring the Brushland Labs Color Rangefinder.
 * Online documentation: <a href="https://docs.brushlandlabs.com">...</a>
 */
class ColorRangefinder {
    private final I2cDeviceSynchSimple i2c;

    public ColorRangefinder(RevColorSensorV3 emulator) {
        this.i2c = emulator.getDeviceClient();
        this.i2c.enableWriteCoalescing(true);
    }

    /**
     * Configure Pin 0 to be in digital mode, and add a threshold.
     * Multiple thresholds can be added to the same pin by calling this function repeatedly.
     * For colors, bounds should be from 0-255, and for distance, bounds should be from 0-100 (mm).
     */
    public void setPin0Digital(DigitalMode digitalMode, double lowerBound, double higherBound) {
        setDigital(PinNum.PIN0, digitalMode, lowerBound, higherBound);
    }

    /**
     * Configure Pin 1 to be in digital mode, and add a threshold.
     * Multiple thresholds can be added to the same pin by calling this function repeatedly.
     * For colors, bounds should be from 0-255, and for distance, bounds should be from 0-100 (mm).
     */
    public void setPin1Digital(DigitalMode digitalMode, double lowerBound, double higherBound) {
        setDigital(PinNum.PIN1, digitalMode, lowerBound, higherBound);
    }

    /**
     * Sets the maximum distance (in millimeters) within which an object must be located for Pin 0's thresholds to trigger.
     * This is most useful when we want to know if an object is both close and the correct color.
     */
    public void setPin0DigitalMaxDistance(DigitalMode digitalMode, double mmRequirement) {
        setPin0Digital(digitalMode, mmRequirement, mmRequirement);
    }

    /**
     * Sets the maximum distance (in millimeters) within which an object must be located for Pin 1's thresholds to trigger.
     * This is most useful when we want to know if an object is both close and the correct color.
     */
    public void setPin1DigitalMaxDistance(DigitalMode digitalMode, double mmRequirement) {
        setPin1Digital(digitalMode, mmRequirement, mmRequirement);
    }

    /**
     * Invert the hue value before thresholding it, meaning that the colors become their opposite.
     * This is useful if we want to threshold red; instead of having two thresholds we would invert
     * the color and look for blue.
     */
    public void setPin0InvertHue() {
        setPin0DigitalMaxDistance(DigitalMode.HSV, 200);
    }

    /**
     * Invert the hue value before thresholding it, meaning that the colors become their opposite.
     * This is useful if we want to threshold red; instead of having two thresholds we would invert
     * the color and look for blue.
     */
    public void setPin1InvertHue() {
        setPin1DigitalMaxDistance(DigitalMode.HSV, 200);
    }

    /**
     * The denominator is what the raw sensor readings will be divided by before being scaled to 12-bit analog.
     * For the full range of that channel, leave the denominator as 65535 for colors or 100 for distance.
     * Smaller values will clip off higher ranges of the data in exchange for higher resolution within a lower range.
     */
    public void setPin0Analog(AnalogMode analogMode, int denominator) {
        byte denom0 = (byte) (denominator & 0xFF);
        byte denom1 = (byte) ((denominator & 0xFF00) >> 8);
        i2c.write(PinNum.PIN0.modeAddress, new byte[]{analogMode.value, denom0, denom1});
    }

    /**
     * Configure Pin 0 as analog output of one of the six data channels.
     * To read analog, make sure the physical switch on the sensor is flipped away from the
     * connector side.
     */
    public void setPin0Analog(AnalogMode analogMode) {
        setPin0Analog(analogMode, analogMode == AnalogMode.DISTANCE ? 100 : 0xFFFF);
    }

    public float[] getCalibration() {
        java.nio.ByteBuffer bytes =
                java.nio.ByteBuffer.wrap(i2c.read(CALIB_A_VAL_0, 16)).order(java.nio.ByteOrder.LITTLE_ENDIAN);
        return new float[]{bytes.getFloat(), bytes.getFloat(), bytes.getFloat(), bytes.getFloat()};
    }

    /**
     * Save a brightness value of the LED to the sensor.
     *
     * @param value brightness between 0-255
     */
    public void setLedBrightness(int value) {
        i2c.write8(LED_BRIGHTNESS, value);
    }

    /**
     * Change the I2C address at which the sensor will be found. The address can be reset to the
     * default of 0x52 by holding the reset button.
     *
     * @param value new I2C address from 1 to 127
     */
    public void setI2cAddress(int value) {
        i2c.write8(I2C_ADDRESS_REG, value << 1);
    }

    /**
     * Read distance via I2C
     * @return distance in millimeters
     */
    public double readDistance() {
        java.nio.ByteBuffer bytes =
                java.nio.ByteBuffer.wrap(i2c.read(PS_DISTANCE_0, 4)).order(java.nio.ByteOrder.LITTLE_ENDIAN);
        return bytes.getFloat();
    }

    private void setDigital(
            PinNum pinNum,
            DigitalMode digitalMode,
            double lowerBound,
            double higherBound
    ) {
        int lo, hi;
        if (lowerBound == higherBound) {
            lo = (int) lowerBound;
            hi = (int) higherBound;
        } else if (digitalMode.value <= DigitalMode.HSV.value) { // color value 0-255
            lo = (int) Math.round(lowerBound / 255.0 * 65535);
            hi = (int) Math.round(higherBound / 255.0 * 65535);
        } else { // distance in mm
            float[] calib = getCalibration();
            if (lowerBound < .5) hi = 2048;
            else hi = rawFromDistance(calib[0], calib[1], calib[2], calib[3], lowerBound);
            lo = rawFromDistance(calib[0], calib[1], calib[2], calib[3], higherBound);
        }

        byte lo0 = (byte) (lo & 0xFF);
        byte lo1 = (byte) ((lo & 0xFF00) >> 8);
        byte hi0 = (byte) (hi & 0xFF);
        byte hi1 = (byte) ((hi & 0xFF00) >> 8);
        i2c.write(pinNum.modeAddress, new byte[]{digitalMode.value, lo0, lo1, hi0, hi1});
        try {
            Thread.sleep(25);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
    }

    private double root(double n, double v) {
        double val = Math.pow(v, 1.0 / Math.abs(n));
        if (n < 0) val = 1.0 / val;
        return val;
    }

    private int rawFromDistance(float a, float b, float c, float x0, double mm) {
        return (int) (root(b, (mm - c) / a) + x0);
    }

    private enum PinNum {
        PIN0(0x28), PIN1(0x2D);

        private final byte modeAddress;

        PinNum(int modeAddress) {
            this.modeAddress = (byte) modeAddress;
        }
    }

    // other writeable registers
    private static final byte CALIB_A_VAL_0 = 0x32;
    private static final byte PS_DISTANCE_0 = 0x42;
    private static final byte LED_BRIGHTNESS = 0x46;
    private static final byte I2C_ADDRESS_REG = 0x47;

    public static int invertHue(int hue360) {
        return ((hue360 - 180) % 360);
    }

    public enum DigitalMode {
        RED(1), BLUE(2), GREEN(3), ALPHA(4), HSV(5), DISTANCE(6);
        public final byte value;

        DigitalMode(int value) {
            this.value = (byte) value;
        }
    }

    public enum AnalogMode {
        RED(13), BLUE(14), GREEN(15), ALPHA(16), HSV(17), DISTANCE(18);
        public final byte value;

        AnalogMode(int value) {
            this.value = (byte) value;
        }
    }
}
