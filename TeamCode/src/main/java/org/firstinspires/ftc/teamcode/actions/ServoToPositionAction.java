package org.firstinspires.ftc.teamcode.actions;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Action that sends a servo to a target position.
 * Completes instantly because we have no feedback on the current position.
 * Combine with a SleepAction to wait while the servo moves!
 */
public class ServoToPositionAction extends StagedAction{


    private Servo servo;
    private double position;

    /**
     * Creates a new servo to position action.
     * @param servo The servo to move
     * @param position The position to send the servo to
     */
    public ServoToPositionAction(Servo servo, double position)
    {
        this.servo = servo;
        this.position = position;
    }

    /**
     * Code in start() will run once when the action starts.
     */
    @Override
    protected void start() {
        servo.setPosition(position);
    }

    /**
     * Code in loop() will run repeatedly until the action is completed.
     * Warning: Don't put any long blocking code (such as wait statements) here!
     *
     * @param telemetryPacket
     */
    @Override
    protected void loop(TelemetryPacket telemetryPacket) {

    }

    /**
     * Code to check when this action is done.
     * Runs right after loop().
     *
     * @return true if the action is finished and should stop, false if it should continue
     */
    @Override
    protected boolean isFinished() {
        return true;
    }

    /**
     * Runs once when the action is finished.
     * Use this to clean up after this action (set motor modes back for teleop, etc).
     * Warning: This won't run if the action is cancelled/interrupted!
     */
    @Override
    protected void end() {

    }
}
