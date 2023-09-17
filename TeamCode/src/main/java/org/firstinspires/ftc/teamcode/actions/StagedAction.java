package org.firstinspires.ftc.teamcode.actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

/**
 * This class expands an Action's lifecycle into multiple methods
 * to make writing custom Actions easier.
 */
public abstract class StagedAction implements Action {

    private boolean isStarted = false;

    /**
     * (Optional) Draw something on the field diagram in FTC Dashboard
     * @param fieldOverlay The field canvas to draw on
     */
    @Override
    public void preview(@NonNull Canvas fieldOverlay) {
        Action.super.preview(fieldOverlay);
    }


    /**
     * Run the action. DO NOT OVERRIDE IN SUBCLASSES!
     */
    @Override
    public final boolean run(@NonNull TelemetryPacket telemetryPacket) {
        // Check if we should run start
        if(isStarted == false)
        {
            start();
            isStarted = true;
            return true;
        }
        else
        {
            // Check if done
            if(isFinished())
            {
                // Done!
                end();
                return false;
            }
            else
            {
                // Not done, keep looping
                loop(telemetryPacket);
                return true;
            }
        }
    }

    /**
     * Code in start() will run once when the action starts.
     */
    protected abstract void start();

    /**
     * Code in loop() will run repeatedly until the action is completed.
     * Warning: Don't put any long blocking code (such as wait statements) here!
     */
    protected abstract void loop(TelemetryPacket telemetryPacket);

    /**
     * Code to check when this action is done.
     * Runs right after loop().
     * @return true if the action is finished and should stop, false if it should continue
     */
    protected abstract boolean isFinished();

    /**
     * Runs once when the action is finished.
     * Use this to clean up after this action (set motor modes back for teleop, etc).
     * Warning: This won't run if the action is cancelled/interrupted!
     */
    protected abstract void end();

}
