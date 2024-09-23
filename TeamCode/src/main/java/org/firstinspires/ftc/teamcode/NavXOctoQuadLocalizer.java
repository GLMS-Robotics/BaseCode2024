package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Twist2dDual;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.Vector2dDual;
import com.acmerobotics.roadrunner.ftc.Encoder;
import com.acmerobotics.roadrunner.ftc.FlightRecorder;
import com.acmerobotics.roadrunner.ftc.PositionVelocityPair;
import com.acmerobotics.roadrunner.ftc.RawEncoder;
import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;
import com.qualcomm.hardware.lynx.LynxDcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Config
public final class NavXOctoQuadLocalizer implements Localizer {
    public static class Params {
        public double parYTicks = 2565.2721757340855; // y position of the parallel encoder (in tick units)
        public double perpXTicks = -2186.730618563984; // x position of the perpendicular encoder (in tick units)
    }

    public static Params PARAMS = new Params();

    public final Encoder par, perp;

    public final NavxMicroNavigationSensor navx;
    public final IntegratingGyroscope imu;

    private int lastParPos, lastPerpPos;
    private Rotation2d lastHeading;

    private final double inPerTick;

    private double lastRawHeadingVel, headingVelOffset;
    private boolean initialized;

    public NavXOctoQuadLocalizer(HardwareMap hardwareMap, double inPerTick) {
        // TODO: Set OctoQuad port numbers
        par = new RawEncoder(new OctoFakeMotor(Robot.octoQuad, 0,
                (LynxDcMotorController) hardwareMap.get(DcMotorEx.class, "FL").getController()));
        perp = new RawEncoder(new OctoFakeMotor(Robot.octoQuad, 1,
                (LynxDcMotorController) hardwareMap.get(DcMotorEx.class, "BL").getController()));

        // TODO: reverse encoder directions if needed
        par.setDirection(DcMotorSimple.Direction.FORWARD);
        perp.setDirection(DcMotorSimple.Direction.REVERSE);

        navx = hardwareMap.get(NavxMicroNavigationSensor.class, "navx");
        this.imu = (IntegratingGyroscope)navx;

        // Wait until the gyro calibration is complete
        try {
            while (navx.isCalibrating()) {
                Thread.sleep(50);
            }
        }
        catch(InterruptedException e)
        {
            // whoops!
        }



        this.inPerTick = inPerTick;

        FlightRecorder.write("TWO_DEAD_WHEEL_PARAMS", PARAMS);
    }

    public Twist2dDual<Time> update() {
        /*PositionVelocityPair parPosVel = new PositionVelocityPair(
                Robot.octoQuad.readSinglePosition_Caching(0),
                Robot.octoQuad.readSingleVelocity_Caching(0),
                0,0);//unused
        //= par.getPositionAndVelocity();
        PositionVelocityPair perpPosVel = new PositionVelocityPair(
                Robot.octoQuad.readSinglePosition_Caching(1),
                Robot.octoQuad.readSingleVelocity_Caching(1),
                0,0);
        //= perp.getPositionAndVelocity();
        */

        PositionVelocityPair parPosVel = par.getPositionAndVelocity();
        PositionVelocityPair perpPosVel = perp.getPositionAndVelocity();

        AngularVelocity rates = imu.getAngularVelocity(AngleUnit.RADIANS);
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);

        //YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();
        //AngularVelocity angularVelocity = imu.getRobotAngularVelocity(AngleUnit.RADIANS);

        //FlightRecorder.write("TWO_DEAD_WHEEL_INPUTS", new TwoDeadWheelInputsMessage(parPosVel, perpPosVel, angles, rates));

        //Rotation2d heading = Rotation2d.exp(angles.getYaw(AngleUnit.RADIANS));
        Rotation2d heading = Rotation2d.exp(angles.firstAngle);

        // see https://github.com/FIRST-Tech-Challenge/FtcRobotController/issues/617
        //double rawHeadingVel = rates.zRotationRate;
        double rawHeadingVel = rates.zRotationRate;
        if (Math.abs(rawHeadingVel - lastRawHeadingVel) > Math.PI) {
            headingVelOffset -= Math.signum(rawHeadingVel) * 2 * Math.PI;
        }
        lastRawHeadingVel = rawHeadingVel;
        double headingVel = headingVelOffset + rawHeadingVel;

        if (!initialized) {
            initialized = true;

            lastParPos = parPosVel.position;
            lastPerpPos = perpPosVel.position;
            lastHeading = heading;

            return new Twist2dDual<>(
                    Vector2dDual.constant(new Vector2d(0.0, 0.0), 2),
                    DualNum.constant(0.0, 2)
            );
        }

        int parPosDelta = parPosVel.position - lastParPos;
        int perpPosDelta = perpPosVel.position - lastPerpPos;
        double headingDelta = heading.minus(lastHeading);

        Twist2dDual<Time> twist = new Twist2dDual<>(
                new Vector2dDual<>(
                        new DualNum<Time>(new double[] {
                                parPosDelta - PARAMS.parYTicks * headingDelta,
                                parPosVel.velocity - PARAMS.parYTicks * headingVel,
                        }).times(inPerTick),
                        new DualNum<Time>(new double[] {
                                perpPosDelta - PARAMS.perpXTicks * headingDelta,
                                perpPosVel.velocity - PARAMS.perpXTicks * headingVel,
                        }).times(inPerTick)
                ),
                new DualNum<>(new double[] {
                        headingDelta,
                        headingVel,
                })
        );

        lastParPos = parPosVel.position;
        lastPerpPos = perpPosVel.position;
        lastHeading = heading;

        return twist;
    }
}
