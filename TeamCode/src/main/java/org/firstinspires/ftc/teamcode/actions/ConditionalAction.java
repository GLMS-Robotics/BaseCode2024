package org.firstinspires.ftc.teamcode.actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import java.util.function.Supplier;

/**
 * This action will decide whether to perform one of two actions based on a condition
 * checked when the action starts running.
 */
public class ConditionalAction implements Action {


    private boolean checked = false;
    private boolean wasTrue;

    private Supplier<Boolean> condition;
    private Action actionIfTrue;
    private Action actionIfFalse;

    /**
     * Create a conditional action.
     * @param condition Condition to check when the action starts running
     * @param actionIfTrue
     * @param actionIfFalse
     */
    public ConditionalAction(Supplier<Boolean> condition, Action actionIfTrue, Action actionIfFalse)
    {
        this.condition = condition;
        this.actionIfFalse = actionIfFalse;
        this.actionIfTrue = actionIfTrue;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        if(!checked)
        {
            wasTrue = condition.get();
            checked = true;
        }

        if(wasTrue)
        {
            return actionIfTrue.run(telemetryPacket);
        }
        else
        {
            return actionIfFalse.run(telemetryPacket);
        }
    }
}
