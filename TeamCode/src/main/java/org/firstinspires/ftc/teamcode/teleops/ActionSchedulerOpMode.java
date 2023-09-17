package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.subsystems.Subsystem;

import java.util.ArrayList;

public abstract class ActionSchedulerOpMode extends OpMode {

    protected final FtcDashboard dash = FtcDashboard.getInstance();

    // List of actions currently running and their subsystems
    private class ActionWithSubsystems {
        public Subsystem[] subsystems;
        public Action action;

        public ActionWithSubsystems(Action a, Subsystem[] s)
        {
            action = a;
            subsystems = s;
        }

    }
    private ArrayList<ActionWithSubsystems> activeActions = new ArrayList<ActionWithSubsystems>();


    @Override
    public void init() {
        Robot.initHardware(hardwareMap);

        initOpMode();
    }

    @Override
    public void loop() {
        // Create telemetry packet and add field map
        TelemetryPacket p = new TelemetryPacket();

        // Run op mode's loop
        loopOpMode(p);

        // Run each subsystem's periodic function
        for(Subsystem s : Robot.subsystems)
        {
            s.periodic(p);
        }

        // Run each active action, and remove it if finished
        activeActions.removeIf(as -> !as.action.run(p));

        // Send telemetry to dashboard
        dash.sendTelemetryPacket(p);
    }

    @Override
    public void stop()
    {
        cancelActions();
    }


    public abstract void initOpMode();

    public abstract void loopOpMode(TelemetryPacket p);

    /**
     * Schedule an action to start running. Needs an action, and an array of subsystems
     * that the action uses (to mark them as busy and prevent double scheduling).
     * @param a The action to run
     * @param usedSubsystems Array of subsystems used by this action
     */
    public void runAction(Action a, Subsystem... usedSubsystems)
    {
        // Make sure these subsystems aren't already busy
        for(Subsystem s : usedSubsystems)
        {
            // Cancel anything already running on this subsystem
            // Could also have it throw an exception or something
            cancelSubsystem(s);
        }

        // Add action to active list
        activeActions.add(new ActionWithSubsystems(a, usedSubsystems));
    }

    /**
     * Cancel all active actions.
     */
    public void cancelActions()
    {
        activeActions.clear();
    }

    /**
     * Cancel action using a particular subsystem, if any
     * @param subsystem The subsystem to cancel
     * @return True if something was cancelled, false otherwise
     */
    public boolean cancelSubsystem(Subsystem subsystem)
    {
        for(ActionWithSubsystems as : activeActions) {
            for (Subsystem s : as.subsystems)
            {
                if (subsystem == s) {
                    // Cancel this action
                    activeActions.remove(as);
                    return true;
                }
            }
        }

        return false;
    }

    /**
     * Check if a subsystem is currently busy performing an action.
     * @param subsystem The subsystem to check
     * @return true if subsystem is busy running an action, false otherwise
     */
    public boolean isBusy(Subsystem subsystem)
    {
        for (ActionWithSubsystems as : activeActions) {
            for(Subsystem s : as.subsystems)
            {
                if(s == subsystem) {
                    return true;
                }
            }
        }

        // Didn't find subsystem in active list
        return false;
    }

}
