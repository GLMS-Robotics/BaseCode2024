package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.digitalchickenlabs.CachingOctoQuad;
import com.qualcomm.hardware.digitalchickenlabs.OctoQuadBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.subsystems.Subsystem;

/**
 * This class should hold static references to all your subsystems,
 * and an array of those subsystems for iteration by other classes.
 *
 * You will use it from your opmodes to access the robot's hardware.
 */
public class Robot {

    public static CachingOctoQuad octoQuad;

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

    /**
     * This method is split out from initHardware so that it can be run from MecanumDrive.
     * That way, the OctoQuad is still initialized in tuning op modes.
     */
    public static void configureOctoquad(HardwareMap hardwareMap) {
        // Set up octoquad
        // 0-3 are for quad encoders, 4-7 are for absolute
        // Also set to manual cache mode
        octoQuad = hardwareMap.get(CachingOctoQuad.class, "octoquad");
        octoQuad.setChannelBankConfig(OctoQuadBase.ChannelBankConfig.BANK1_QUADRATURE_BANK2_PULSE_WIDTH);
        octoQuad.setAllVelocitySampleIntervals(25);
        for(int j=4; j<8; j++)
        {
            // Set PWM to match rev through bore encoders
            octoQuad.setSingleChannelPulseWidthParams (j, 1,1024);
        }

        // Would have preferred to use manual, but can't seem to sneak in a refresh call for RR tuning
        octoQuad.setCachingMode(CachingOctoQuad.CachingMode.AUTO);

        // Called from MecanumDrive instead so that encoder reversals are saved
        //octoQuad.saveParametersToFlash();
    }


}
