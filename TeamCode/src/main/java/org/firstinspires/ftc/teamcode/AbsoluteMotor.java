package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.digitalchickenlabs.CachingOctoQuad;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

/**
 * Wrapper for a DcMotorEx that adds absolute encoder functionality via an OctoQuad.
 */
public class AbsoluteMotor implements DcMotorEx {

    // Hardware
    private DcMotorEx baseMotor;
    private CachingOctoQuad octoQuad;
    private int octoQuadPort;

    // Absolute encoder data for calculations
    private double offsetPercent, rolloverPercent, motorTicksPerRev;
    private int absMin, absMax;
    private boolean absReversed;

    // Current difference between absolute encoder's calculated position and motor encoder's position
    private double currentOffsetTicks = 0;

    // Track targeted encoder position to update it if the offset changes
    private int lastTargetPos = 0;

    public AbsoluteMotor(DcMotorEx baseMotor, CachingOctoQuad octoQuad, int octoQuadPort,
                         double offsetPercent, double rolloverPercent,
                         int absMin, int absMax, double motorTicksPerRev, boolean absReversed) {
        this.baseMotor = baseMotor;
        this.octoQuad = octoQuad;
        this.octoQuadPort = octoQuadPort;
        this.offsetPercent = offsetPercent;
        this.rolloverPercent = rolloverPercent;
        this.absMin = absMin;
        this.absMax = absMax;
        this.motorTicksPerRev = motorTicksPerRev;
        this.absReversed = absReversed;

        resetInternalEncoderToAbsolute();
    }

    /**
     * Read the absolute encoder and use it to correct the motor's position.
     */
    public void resetInternalEncoderToAbsolute() {
        currentOffsetTicks =  getAbsolutePosition() - baseMotor.getCurrentPosition();
    }

    public double getAbsolutePosition()
    {
        return convertAbsoluteToMotorEncoder(octoQuad.readSinglePosition_Caching(octoQuadPort));
    }

    /**
     * Convert an absolute position to a position in motor encoder units.
     * @param position
     * @return
     */
    private double convertAbsoluteToMotorEncoder(int position)
    {
        // Scale to 0-1
        double pos = (double) (position - absMin) / absMax;

        // Offset to 0 point
        pos -= offsetPercent;

        // Roll over at rollover target
        if(pos > rolloverPercent) {
            pos -= 1;
        }
        if(position < rolloverPercent-1)
        {
            pos += 1;
        }

        // Scale position back up
        pos *= motorTicksPerRev * (absReversed ? -1 : 1);

        return pos;
    }

    /**
     * Individually energizes this particular motor
     *
     * @see #setMotorDisable()
     * @see #isMotorEnabled()
     */
    @Override
    public void setMotorEnable() {
        baseMotor.setMotorEnable();
    }

    /**
     * Individually de-energizes this particular motor
     *
     * @see #setMotorEnable()
     * @see #isMotorEnabled()
     */
    @Override
    public void setMotorDisable() {
        baseMotor.setMotorDisable();
    }

    /**
     * Returns whether this motor is energized
     *
     * @see #setMotorEnable()
     * @see #setMotorDisable()
     */
    @Override
    public boolean isMotorEnabled() {
        return baseMotor.isMotorEnabled();
    }

    /**
     * Sets the velocity of the motor
     *
     * @param angularRate the desired ticks per second
     */
    @Override
    public void setVelocity(double angularRate) {
        baseMotor.setVelocity(angularRate);
    }

    /**
     * Sets the velocity of the motor
     *
     * @param angularRate the desired angular rate, in units per second
     * @param unit        the units in which angularRate is expressed
     * @see #getVelocity(AngleUnit)
     */
    @Override
    public void setVelocity(double angularRate, AngleUnit unit) {
        baseMotor.setVelocity(angularRate, unit);
    }

    /**
     * Returns the current velocity of the motor, in ticks per second
     *
     * @return the current velocity of the motor
     */
    @Override
    public double getVelocity() {
        return baseMotor.getVelocity();
    }

    /**
     * Returns the current velocity of the motor, in angular units per second
     *
     * @param unit the units in which the angular rate is desired
     * @return the current velocity of the motor
     * @see #setVelocity(double, AngleUnit)
     */
    @Override
    public double getVelocity(AngleUnit unit) {
        return baseMotor.getVelocity(unit);
    }

    /**
     * Sets the PID control coefficients for one of the PID modes of this motor.
     * Note that in some controller implementations, setting the PID coefficients for one
     * mode on a motor might affect other modes on that motor, or might affect the PID
     * coefficients used by other motors on the same controller (this is not true on the
     * REV Expansion Hub).
     *
     * @param mode            either {@link RunMode#RUN_USING_ENCODER} or {@link RunMode#RUN_TO_POSITION}
     * @param pidCoefficients the new coefficients to use when in that mode on this motor
     * @see #getPIDCoefficients(RunMode)
     * @deprecated Use {@link #setPIDFCoefficients(RunMode, PIDFCoefficients)} instead
     */
    @Override
    public void setPIDCoefficients(RunMode mode, PIDCoefficients pidCoefficients) {
        baseMotor.setPIDCoefficients(mode, pidCoefficients);
    }

    /**
     * {@link #setPIDFCoefficients} is a superset enhancement to {@link #setPIDCoefficients}. In addition
     * to the proportional, integral, and derivative coefficients previously supported, a feed-forward
     * coefficient may also be specified. Further, a selection of motor control algorithms is offered:
     * the originally-shipped Legacy PID algorithm, and a PIDF algorithm which avails itself of the
     * feed-forward coefficient. Note that the feed-forward coefficient is not used by the Legacy PID
     * algorithm; thus, the feed-forward coefficient must be indicated as zero if the Legacy PID
     * algorithm is used. Also: the internal implementation of these algorithms may be different: it
     * is not the case that the use of PIDF with the F term as zero necessarily exhibits exactly the
     * same behavior as the use of the LegacyPID algorithm, though in practice they will be quite close.
     * <p>
     * Readers are reminded that {@link DcMotor.RunMode#RUN_TO_POSITION} mode makes use of <em>both</em>
     * the coefficients set for RUN_TO_POSITION <em>and</em> the coefficients set for RUN_WITH_ENCODER,
     * due to the fact that internally the RUN_TO_POSITION logic calculates an on-the-fly velocity goal
     * on each control cycle, then (logically) runs the RUN_WITH_ENCODER logic. Because of that double-
     * layering, only the proportional ('p') coefficient makes logical sense for use in the RUN_TO_POSITION
     * coefficients.
     *
     * @param mode
     * @param pidfCoefficients
     * @see #setVelocityPIDFCoefficients(double, double, double, double)
     * @see #setPositionPIDFCoefficients(double)
     * @see #getPIDFCoefficients(RunMode)
     */
    @Override
    public void setPIDFCoefficients(RunMode mode, PIDFCoefficients pidfCoefficients) throws UnsupportedOperationException {
        baseMotor.setPIDFCoefficients(mode, pidfCoefficients);
    }

    /**
     * A shorthand for setting the PIDF coefficients for the {@link DcMotor.RunMode#RUN_USING_ENCODER}
     * mode. {@link MotorControlAlgorithm#PIDF} is used.
     *
     * @param p
     * @param i
     * @param d
     * @param f
     * @see #setPIDFCoefficients(RunMode, PIDFCoefficients)
     */
    @Override
    public void setVelocityPIDFCoefficients(double p, double i, double d, double f) {
        setVelocityPIDFCoefficients(p,i,d,f);
    }

    /**
     * A shorthand for setting the PIDF coefficients for the {@link DcMotor.RunMode#RUN_TO_POSITION}
     * mode. {@link MotorControlAlgorithm#PIDF} is used.
     * <p>
     * Readers are reminded that {@link DcMotor.RunMode#RUN_TO_POSITION} mode makes use of <em>both</em>
     * the coefficients set for RUN_TO_POSITION <em>and</em> the coefficients set for RUN_WITH_ENCODER,
     * due to the fact that internally the RUN_TO_POSITION logic calculates an on-the-fly velocity goal
     * on each control cycle, then (logically) runs the RUN_WITH_ENCODER logic. Because of that double-
     * layering, only the proportional ('p') coefficient makes logical sense for use in the RUN_TO_POSITION
     * coefficients.
     *
     * @param p
     * @see #setVelocityPIDFCoefficients(double, double, double, double)
     * @see #setPIDFCoefficients(RunMode, PIDFCoefficients)
     */
    @Override
    public void setPositionPIDFCoefficients(double p) {
        baseMotor.setPositionPIDFCoefficients(p);
    }

    /**
     * Returns the PID control coefficients used when running in the indicated mode
     * on this motor.
     *
     * @param mode either {@link RunMode#RUN_USING_ENCODER} or {@link RunMode#RUN_TO_POSITION}
     * @return the PID control coefficients used when running in the indicated mode on this motor
     * @deprecated Use {@link #getPIDFCoefficients(RunMode)} instead
     */
    @Override
    public PIDCoefficients getPIDCoefficients(RunMode mode) {
        return baseMotor.getPIDCoefficients(mode);
    }

    /**
     * Returns the PIDF control coefficients used when running in the indicated mode
     * on this motor.
     *
     * @param mode either {@link RunMode#RUN_USING_ENCODER} or {@link RunMode#RUN_TO_POSITION}
     * @return the PIDF control coefficients used when running in the indicated mode on this motor
     * @see #setPIDFCoefficients(RunMode, PIDFCoefficients)
     */
    @Override
    public PIDFCoefficients getPIDFCoefficients(RunMode mode) {
        return baseMotor.getPIDFCoefficients(mode);
    }

    /**
     * Sets the target positioning tolerance of this motor
     *
     * @param tolerance the desired tolerance, in encoder ticks
     * @see DcMotor#setTargetPosition(int)
     */
    @Override
    public void setTargetPositionTolerance(int tolerance) {
        baseMotor.setTargetPositionTolerance(tolerance);
    }

    /**
     * Returns the current target positioning tolerance of this motor
     *
     * @return the current target positioning tolerance of this motor
     */
    @Override
    public int getTargetPositionTolerance() {
        return baseMotor.getTargetPositionTolerance();
    }

    /**
     * Returns the current consumed by this motor.
     *
     * @param unit current units
     * @return the current consumed by this motor.
     */
    @Override
    public double getCurrent(CurrentUnit unit) {
        return baseMotor.getCurrent(unit);
    }

    /**
     * Returns the current alert for this motor.
     *
     * @param unit current units
     * @return the current alert for this motor
     */
    @Override
    public double getCurrentAlert(CurrentUnit unit) {
        return baseMotor.getCurrentAlert(unit);
    }

    /**
     * Sets the current alert for this motor
     *
     * @param current current alert
     * @param unit    current units
     */
    @Override
    public void setCurrentAlert(double current, CurrentUnit unit) {
        baseMotor.setCurrentAlert(current, unit);
    }

    /**
     * Returns whether the current consumption of this motor exceeds the alert threshold.
     *
     * @return whether the current consumption of this motor exceeds the alert threshold.
     */
    @Override
    public boolean isOverCurrent() {
        return baseMotor.isOverCurrent();
    }

    /**
     * Returns the assigned type for this motor. If no particular motor type has been
     * configured, then {@link MotorConfigurationType#getUnspecifiedMotorType()} will be returned.
     * Note that the motor type for a given motor is initially assigned in the robot
     * configuration user interface, though it may subsequently be modified using methods herein.
     *
     * @return the assigned type for this motor
     */
    @Override
    public MotorConfigurationType getMotorType() {
        return baseMotor.getMotorType();
    }

    /**
     * Sets the assigned type of this motor. Usage of this method is very rare.
     *
     * @param motorType the new assigned type for this motor
     * @see #getMotorType()
     */
    @Override
    public void setMotorType(MotorConfigurationType motorType) {
        baseMotor.setMotorType(motorType);
    }

    /**
     * Returns the underlying motor controller on which this motor is situated.
     *
     * @return the underlying motor controller on which this motor is situated.
     * @see #getPortNumber()
     */
    @Override
    public DcMotorController getController() {
        return baseMotor.getController();
    }

    /**
     * Returns the port number on the underlying motor controller on which this motor is situated.
     *
     * @return the port number on the underlying motor controller on which this motor is situated.
     * @see #getController()
     */
    @Override
    public int getPortNumber() {
        return baseMotor.getPortNumber();
    }

    /**
     * Sets the behavior of the motor when a power level of zero is applied.
     *
     * @param zeroPowerBehavior the new behavior of the motor when a power level of zero is applied.
     * @see ZeroPowerBehavior
     * @see #setPower(double)
     */
    @Override
    public void setZeroPowerBehavior(ZeroPowerBehavior zeroPowerBehavior) {
        baseMotor.setZeroPowerBehavior(zeroPowerBehavior);
    }

    /**
     * Returns the current behavior of the motor were a power level of zero to be applied.
     *
     * @return the current behavior of the motor were a power level of zero to be applied.
     */
    @Override
    public ZeroPowerBehavior getZeroPowerBehavior() {
        return baseMotor.getZeroPowerBehavior();
    }

    /**
     * Sets the zero power behavior of the motor to {@link ZeroPowerBehavior#FLOAT FLOAT}, then
     * applies zero power to that motor.
     *
     * <p>Note that the change of the zero power behavior to {@link ZeroPowerBehavior#FLOAT FLOAT}
     * remains in effect even following the return of this method. <STRONG>This is a breaking
     * change</STRONG> in behavior from previous releases of the SDK. Consider, for example, the
     * following code sequence:</p>
     *
     * <pre>
     *     motor.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE); // method not available in previous releases
     *     motor.setPowerFloat();
     *     motor.setPower(0.0);
     * </pre>
     *
     * <p>Starting from this release, this sequence of code will leave the motor floating. Previously,
     * the motor would have been left braked.</p>
     *
     * @see #setPower(double)
     * @see #getPowerFloat()
     * @see #setZeroPowerBehavior(ZeroPowerBehavior)
     * @deprecated This method is deprecated in favor of direct use of
     * {@link #setZeroPowerBehavior(ZeroPowerBehavior) setZeroPowerBehavior()} and
     * {@link #setPower(double) setPower()}.
     */
    @Override
    public void setPowerFloat() {
        baseMotor.setPowerFloat();
    }

    /**
     * Returns whether the motor is currently in a float power level.
     *
     * @return whether the motor is currently in a float power level.
     * @see #setPowerFloat()
     */
    @Override
    public boolean getPowerFloat() {
        return baseMotor.getPowerFloat();
    }

    /**
     * Sets the desired encoder target position to which the motor should advance or retreat
     * and then actively hold thereat. This behavior is similar to the operation of a servo.
     * The maximum speed at which this advance or retreat occurs is governed by the power level
     * currently set on the motor. While the motor is advancing or retreating to the desired
     * taget position, {@link #isBusy()} will return true.
     *
     * <p>Note that adjustment to a target position is only effective when the motor is in
     * {@link RunMode#RUN_TO_POSITION RUN_TO_POSITION}
     * RunMode. Note further that, clearly, the motor must be equipped with an encoder in order
     * for this mode to function properly.</p>
     *
     * @param position the desired encoder target position
     * @see #getCurrentPosition()
     * @see #setMode(RunMode)
     * @see RunMode#RUN_TO_POSITION
     * @see #getTargetPosition()
     * @see #isBusy()
     */
    @Override
    public void setTargetPosition(int position) {
        lastTargetPos = position;
        baseMotor.setTargetPosition((int) (position-currentOffsetTicks));
    }

    /**
     * Returns the current target encoder position for this motor.
     *
     * @return the current target encoder position for this motor.
     * @see #setTargetPosition(int)
     */
    @Override
    public int getTargetPosition() {
        return lastTargetPos;
    }

    /**
     * Returns true if the motor is currently advancing or retreating to a target position.
     *
     * @return true if the motor is currently advancing or retreating to a target position.
     * @see #setTargetPosition(int)
     */
    @Override
    public boolean isBusy() {
        return baseMotor.isBusy();
    }

    /**
     * Returns the current reading of the encoder for this motor. The units for this reading,
     * that is, the number of ticks per revolution, are specific to the motor/encoder in question,
     * and thus are not specified here.
     *
     * @return the current reading of the encoder for this motor
     * @see #getTargetPosition()
     * @see RunMode#STOP_AND_RESET_ENCODER
     */
    @Override
    public int getCurrentPosition() {
        return (int) (baseMotor.getCurrentPosition() + currentOffsetTicks);
    }

    /**
     * Sets the current run mode for this motor
     *
     * Note: Also re-reads absolute position!
     *
     * @param mode the new current run mode for this motor
     * @see RunMode
     * @see #getMode()
     */
    @Override
    public void setMode(RunMode mode) {
        baseMotor.setMode(mode);

        if(mode==RunMode.STOP_AND_RESET_ENCODER)
        {
            resetInternalEncoderToAbsolute();
        }
    }

    /**
     * Returns the current run mode for this motor
     *
     * @return the current run mode for this motor
     * @see RunMode
     * @see #setMode(RunMode)
     */
    @Override
    public RunMode getMode() {
        return baseMotor.getMode();
    }

    /**
     * Sets the logical direction in which this motor operates.
     *
     * @param direction the direction to set for this motor
     * @see #getDirection()
     */
    @Override
    public void setDirection(Direction direction) {
        baseMotor.setDirection(direction);
    }

    /**
     * Returns the current logical direction in which this motor is set as operating.
     *
     * @return the current logical direction in which this motor is set as operating.
     * @see #setDirection(Direction)
     */
    @Override
    public Direction getDirection() {
        return baseMotor.getDirection();
    }

    /**
     * Sets the power level of the motor, expressed as a fraction of the maximum
     * possible power / speed supported according to the run mode in which the
     * motor is operating.
     *
     * <p>Setting a power level of zero will brake the motor</p>
     *
     * @param power the new power level of the motor, a value in the interval [-1.0, 1.0]
     * @see #getPower()
     * @see DcMotor#setMode(DcMotor.RunMode)
     * @see DcMotor#setPowerFloat()
     */
    @Override
    public void setPower(double power) {
        baseMotor.setPower(power);
    }

    /**
     * Returns the current configured power level of the motor.
     *
     * @return the current level of the motor, a value in the interval [0.0, 1.0]
     * @see #setPower(double)
     */
    @Override
    public double getPower() {
        return baseMotor.getPower();
    }

    /**
     * Returns an indication of the manufacturer of this device.
     *
     * @return the device's manufacturer
     */
    @Override
    public Manufacturer getManufacturer() {
        return baseMotor.getManufacturer();
    }

    /**
     * Returns a string suitable for display to the user as to the type of device.
     * Note that this is a device-type-specific name; it has nothing to do with the
     * name by which a user might have configured the device in a robot configuration.
     *
     * @return device manufacturer and name
     */
    @Override
    public String getDeviceName() {
        return baseMotor.getDeviceName();
    }

    /**
     * Get connection information about this device in a human readable format
     *
     * @return connection info
     */
    @Override
    public String getConnectionInfo() {
        return baseMotor.getConnectionInfo();
    }

    /**
     * Version
     *
     * @return get the version of this device
     */
    @Override
    public int getVersion() {
        return baseMotor.getVersion();
    }

    /**
     * Resets the device's configuration to that which is expected at the beginning of an OpMode.
     * For example, motors will reset the their direction to 'forward'.
     */
    @Override
    public void resetDeviceConfigurationForOpMode() {
        baseMotor.resetDeviceConfigurationForOpMode();
    }

    /**
     * Closes this device
     */
    @Override
    public void close() {
        baseMotor.close();
    }
}
