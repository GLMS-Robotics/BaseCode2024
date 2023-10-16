package org.firstinspires.ftc.teamcode.autos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SleepAction;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.Subsystem;

/**
 * This class provides a simplified way to program a RoadRunner action-based autonomous.
 * It also supports calling periodic methods defined by the Subystem class.
 *
 * Subclasses simply need to override the autonomousActions method to return the actions
 * to be run during this auto. This will usually be the SequentialAction returned by
 * the drivetrain's actionBuilder method, or a custom SequentialAction.
 */
public abstract class ActionAutonomous extends OpMode {

    // The action this Autonomous will run
    private Action autoAction;

    // Dashboard stuff to show auto path
    private Canvas c = new Canvas();
    private final FtcDashboard dash = FtcDashboard.getInstance();


    @Override
    public void init() {
        Robot.initHardware(hardwareMap);

        // Set robot starting position
        getDrivetrain().updatePoseEstimate();
        getDrivetrain().pose = getStartingPose();

        autoAction = autonomousActions();
        autoAction.preview(c);

        // Loop init action until it finishes
        Action initAction = initActions();
        boolean keepRunning = false;

        do {
            // Create telemetry packet and add field map
            TelemetryPacket p = new TelemetryPacket();
            p.fieldOverlay().getOperations().addAll(c.getOperations());

            // Run action and check if finished
            keepRunning = initAction.run(p);

            dash.sendTelemetryPacket(p);
        } while(keepRunning);
    }

    @Override
    public void loop() {

        // Create telemetry packet and add field map
        TelemetryPacket p = new TelemetryPacket();
        p.fieldOverlay().getOperations().addAll(c.getOperations());

        // Run action and check if finished
        boolean keepRunning = autoAction.run(p);

        for(Subsystem s : Robot.subsystems)
        {
            s.periodic(p);
        }

        // Send telemetry to dashboard
        dash.sendTelemetryPacket(p);

        // End the op mode if it is finished
        if(keepRunning == false)
        {
            requestOpModeStop();
        }
    }

    /**
     * Opmodes should override this function to set actions that should be run
     * when the opmode is initialized (after hardware initialization).
     * @return an Action to be run on init
     */
    public Action initActions()
    {
        return new SleepAction(0);
    }


    /**
     * Opmodes should define the action to be taken during autonomous here.
     * Usually, this should be done using Robot.mecanumDrive's actionBuilder
     * @return an Action representing autonomous (probably built by TrajectoryActionBuilder in
     */
    public abstract Action autonomousActions();


    /**
     * Return a reference to your robot's drivetrain in this function.
     * This will be used to set the starting pose.
     */
    public abstract MecanumDrive getDrivetrain();


    /**
     * This function should return your robot's starting position for this autonomous.
     * This will only be used if this opmode is actually run, and ignored if
     * this auto is used in other opmodes.
     */
    public abstract Pose2d getStartingPose();



}
