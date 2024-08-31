package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.subsystems.Subsystem;

/**
 * This class should hold static references to all your subsystems,
 * and an array of those subsystems for iteration by other classes.
 *
 * You will use it from your opmodes to access the robot's hardware.
 */
public class Robot {

    // TODO Your Subsystems Here
    // For example:
    // public static MySubsystem mySubsystem;


    public static MecanumDrive mecanumDrive;
    
    // We also need an array of subsystems
    // Fill this in in the initHardware method below
    public static Subsystem[] subsystems;

    /**
     * Set up the robot's hardware, if it isn't set up yet.
     */
    public static void initHardware(HardwareMap hardwareMap)
    {
        // We want to keep the current pose if this isn't our first op mode
        Pose2d startPose;
        if(mecanumDrive != null)
            startPose = mecanumDrive.pose;
        else
            startPose = new Pose2d(0,0,0);

        mecanumDrive = new MecanumDrive(hardwareMap, startPose);


        // TODO Initialize your subsystems
        // For example:
        // mySubsystem = new mySubsystem(hardwareMap);

        subsystems = new Subsystem[] {
                // TODO List your subsystems here, putting a comma after each one except the last
                // For example:
                // mySubsystem,
                mecanumDrive
        };

    }


}
