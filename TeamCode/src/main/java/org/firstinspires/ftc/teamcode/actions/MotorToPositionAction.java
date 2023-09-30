package org.firstinspires.ftc.teamcode.actions;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

/**
 * Action that sends a motor to a target position using RUN_TO_POSITION mode.
 * Completes when motor enters error range.
 */
public class MotorToPositionAction extends StagedAction{


    private DcMotorEx motor;
    private int position;
    private int error;
    private double maxSpeed;

    /**
     * Creates a new motor to position action.
     * @param motor The motor to control
     * @param position The target position to go to
     * @param error How close we need to get before the action is complete
     * @param maxSpeed Maximum movement speed
     */
    public MotorToPositionAction(DcMotorEx motor, int position, int error, double maxSpeed)
    {
        this.motor = motor;
        this.position = position;
        this.error = error;
        this.maxSpeed = maxSpeed;
    }

    /**
     * Code in start() will run once when the action starts.
     */
    @Override
    protected void start() {
        motor.setTargetPositionTolerance(error);
        motor.setTargetPosition(position);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(maxSpeed);
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
        return !motor.isBusy();

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
