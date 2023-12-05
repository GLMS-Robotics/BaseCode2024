package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Wrapper class for the REV Digital LED Indicator. The LED can be three colors:
 * - Red
 * - Green
 * - Orange
 */
public class RevLed {

    private DigitalChannel greenLed;
    private DigitalChannel redLed;

    public RevLed(HardwareMap hardwareMap, String redChannel, String greenChannel)
    {
        greenLed = hardwareMap.get(DigitalChannel.class, greenChannel);
        redLed = hardwareMap.get(DigitalChannel.class, redChannel);
        greenLed.setMode(DigitalChannel.Mode.OUTPUT);
        redLed.setMode(DigitalChannel.Mode.OUTPUT);

    }

    /**
     * Set the LED to the color Red
     */
    public void setRed()
    {
        greenLed.setState(false);
        redLed.setState(true);
    }

    /**
     * Set the LED to the color Green
     */
    public void setGreen()
    {
        greenLed.setState(true);
        redLed.setState(false);
    }

    /**
     * Set the LED to the color Orange
     */
    public void setOrange()
    {
        greenLed.setState(true);
        redLed.setState(true);
    }

    /**
     * Turn off the LED
     */
    public void setOff()
    {
        greenLed.setState(true);
        redLed.setState(true);
    }

    /**
     * Action to set the LED to the color Red
     *
     * @return A RoadRunner action that will change this LED to Red.
     */
    public Action setRedAction()
    {
        return telemetryPacket -> {
            setRed();
            return false;
        };
    }

    /**
     * Action to turn off the LED
     *
     * @return A RoadRunner action that will turn off the LED.
     */
    public Action setOffAction()
    {
        return telemetryPacket -> {
            setOff();
            return false;
        };
    }

    /**
     * Action to set the LED to the color Green
     *
     * @return A RoadRunner action that will change this LED to Green.
     */
    public Action setGreenAction()
    {
        return telemetryPacket -> {
            setGreen();
            return false;
        };
    }

    /**
     * Action to set the LED to the color Orange
     *
     * @return A RoadRunner action that will change this LED to Orange.
     */
    public Action setOrangeAction()
    {
        return telemetryPacket -> {
            setOrange();
            return false;
        };
    }






}
