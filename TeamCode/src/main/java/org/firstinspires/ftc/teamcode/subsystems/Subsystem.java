package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
*   Base subsystem class representing a subsystem of the robot.
*   Each subsystem should be a part of the robot that will act independently.
*
*   The subsystem should have instance variables for motors, servos, sensors, etc.
*
*   A subsystem can run RoadRunner actions, but only one at a time.
*   Code that keeps track of and runs Actions lives in the opmode, since
*   it will differ between autonomous and teleops.
*
*   Generally, controls should be programmed in the opmode, while the
*   motors should be controlled here using a method.
*
 */
public abstract class Subsystem {

    // Every subsystem should have a constructor that takes a hardwareMap input
    //public Subsystem(HardwareMap hardwareMap) {}



    /**
     * Method that runs each time the opmode loops.
     * Usually, you will want to output telemetry here (encoder values and such).
     * Warning: Actions may be running, so generally you shouldn't control
     * actuators here, unless you are doing something fun like a state machine
     * (in which case you probably shouldn't control actuators outside of this method...).
     * Instead, have your autonomous run actions, and your teleop schedule actions or run methods
     * directly.
     *
     * @param p Telemetry packet for sending data to the driver hub / dashboard
     */
    public abstract void periodic(TelemetryPacket p);


}
