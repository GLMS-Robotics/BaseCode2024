package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.subsystems.Subsystem;

public abstract class ActionAutonomous extends OpMode {

    // The action this Autonomous will run
    private Action autoAction;

    // Dashboard stuff to show auto path
    private Canvas c = new Canvas();
    private final FtcDashboard dash = FtcDashboard.getInstance();


    @Override
    public void init() {
        Robot.initHardware(hardwareMap);

        autoAction = autonomousActions();
        autoAction.preview(c);
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
     * Opmodes should define the action to be taken during autonomous here.
     * Usually, this should be done using Robot.mecanumDrive's actionBuilder
     * @return an Action representing autonomous (probably built by TrajectoryActionBuilder in
     */
    public abstract Action autonomousActions();
}
