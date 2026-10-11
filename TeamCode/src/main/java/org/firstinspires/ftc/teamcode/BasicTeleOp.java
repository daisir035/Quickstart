package org.firstinspires.ftc.teamcode;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.math.Pose;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.pedro.Constants;

/**
 * Robot-centric teleop ("headed" mode): sticks are relative to the ROBOT's frame.
 * Left stick: drive / strafe      Right stick X: turn
 * Right trigger held: slow mode
 *
 * VELOCITY_CONTROL = true runs the drive motors in closed-loop velocity mode
 * (PID inside the REV hub firmware, uses the HD Hex built-in encoders).
 */
@TeleOp(name = "Basic TeleOp", group = "TeleOp")
public class BasicTeleOp extends LinearOpMode {
    private static final double STICK_DEADZONE = 0.05;
    private static final double SLOW_MODE_SCALE = 0.4;
    private static final double STRAFE_SIGN = -1.0; // flip to -1.0 if strafe is mirrored
    private static final double TURN_SIGN = -1.0;   // flip to -1.0 if turn is inverted
    private static final boolean VELOCITY_CONTROL = true;

    @Override
    public void runOpMode() {
        Mecanum drive = new Mecanum(hardwareMap, Constants.drivetrainConfig);
        PinpointLocalizer localizer = new PinpointLocalizer(hardwareMap, Constants.localizerConfig);

        if (VELOCITY_CONTROL) enableVelocityControl();

        telemetry.addLine("Ready (robot-centric).");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            localizer.update();

            double speed = gamepad1.right_trigger > 0.3 ? SLOW_MODE_SCALE : 1.0;
            double forward = applyDeadzone(-gamepad1.left_stick_y) * speed;
            double strafe = applyDeadzone(gamepad1.left_stick_x) * STRAFE_SIGN * speed;
            double turn = applyDeadzone(gamepad1.right_stick_x) * TURN_SIGN * speed;

            drive.drive(new DrivePowers(forward, strafe, turn), true);

            Pose p = localizer.pose();
            telemetry.addData("x (in)", "%.2f", p.x());
            telemetry.addData("y (in)", "%.2f", p.y());
            telemetry.addData("heading (deg)", "%.1f", Math.toDegrees(p.heading()));
            telemetry.addData("slow mode", speed < 1.0);
            telemetry.update();
        }

        drive.stop();
    }

    /** Closed-loop velocity mode: setPower() becomes a target-velocity command
     *  handled by the velocity PID running in the REV hub firmware. */
    private void enableVelocityControl() {
        String[] names = {
                Constants.drivetrainConfig.frontLeftName.get(),
                Constants.drivetrainConfig.frontRightName.get(),
                Constants.drivetrainConfig.backLeftName.get(),
                Constants.drivetrainConfig.backRightName.get()
        };
        for (String name : names) {
            DcMotorEx motor = hardwareMap.get(DcMotorEx.class, name);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    private static double applyDeadzone(double v) {
        return Math.abs(v) < STICK_DEADZONE ? 0.0 : v;
    }
}
