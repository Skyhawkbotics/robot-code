package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareDeviceCloseOnTearDown;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.tuning.TuningOpModes;


//built from LocalizationTest, but adding the arm stuff
@TeleOp(name="MAIN_opmode")
public class opmode_MAIN extends LinearOpMode {

    //setup arm variable
    private DcMotorEx up;
    private CRServo servo_CLAW;

    int arm_upper_lim = 6000;
    double servo_CLAW_power = 0.0;
    double servo_CLAW_position = 0.0;
    double manualOutControl = 0;
    int up_true_target_pos;

    //time stuff
    double last_time = 0;
    private ElapsedTime     runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

            //setup arm to use velocity
            up = hardwareMap.get(DcMotorEx.class, "up");
            up.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            up.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            up.setDirection(DcMotorSimple.Direction.REVERSE);

            //example position setup
            //out = hardwareMap.get(DcMotorEx.class, "out");
            //out.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            //out.setTargetPosition(0);
            //out.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            //example velocity setup
            //up = hardwareMap.get(DcMotorEx.class, "up");
            //up.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            //up.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            //up.setDirection(DcMotorSimple.Direction.REVERSE);

        servo_CLAW = hardwareMap.get(CRServo.class, "claw");

        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
            MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

            waitForStart();

            while (opModeIsActive()) {


                //driving code taken from LocalizationTest
                drive.setDrivePowers(new PoseVelocity2d(
                        new Vector2d(
                                -gamepad1.left_stick_y,
                                -gamepad1.left_stick_x
                        ),
                        -gamepad1.right_stick_x
                ));

                drive.updatePoseEstimate();

                //arm code
                if (up.getCurrentPosition() < arm_upper_lim && gamepad2.dpad_up) {
                    //use velocity mode to move so it doesn't we all funky with the smoothing of position mode
                    up.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    up.setVelocity(2796);
                    up_true_target_pos = 0;
                }
                else if ( up.getCurrentPosition() > 0 && gamepad2.dpad_down) {
                    up.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    up.setVelocity(-2796);
                    up_true_target_pos = 0;
                } else {
                    up.setPower(500);
                    //use positon mode to stay up, as otherwise it would fall down. do some fancy stuff with up_true_target_pos to avoid the issue of it very slightly falling every tick
                    if (up_true_target_pos == 0) {
                        up.setTargetPosition(up.getCurrentPosition());
                        up_true_target_pos = up.getCurrentPosition();
                    }
                    up.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                }

                //make sure the upper and lower limits are actually at the upper and lower limits
                if (up.getCurrentPosition() < 0) {
                    up.setTargetPosition(0);
                } else if (up.getCurrentPosition() > arm_upper_lim) {
                    up.setTargetPosition(arm_upper_lim);
                }
/*
                //manual out control
                if (gamepad2.left_bumper) {
                    manualOutControl += 1000 * (runtime.seconds() - last_time);
                } else if ((gamepad2.left_trigger > 0.8f)) {
                    manualOutControl += -1000 * (runtime.seconds() - last_time);
                }

                /reset manual out control
                if (gamepad2.a) {
                    manualOutControl = 0;
                }*/

                //out.setVelocity(500);
                // Keep the current expression of arm*2
                // add onto the expression: + myVariable
                // Increment myVariable so u have manual control
                // To "return" back to normal business set it to 0
                //out.setTargetPosition(((int) ((arm.getCurrentPosition() * -0.6) + manualOutControl)));

                // Gamepad2.right_trigger is analog, so we need a compatative statment to use it as a digital button.
                if (gamepad2.right_trigger > 0.8/* && servo_CLAW_position < 1000000000*/) { //TODO: find a better solution for this limits so we can actually use them
                    servo_CLAW_power = 1;
                    servo_CLAW_position += 1 * (runtime.seconds() - last_time);
                } else if (gamepad2.right_bumper/* && servo_CLAW_position > -100000000*/) { //TODO: these limits too.
                    servo_CLAW_power = -1;
                    servo_CLAW_position += -1 * (runtime.seconds() - last_time);
                } else {
                    servo_CLAW_power = 0;
                }

                servo_CLAW.setPower(servo_CLAW_power);



                //telemetry stuff (prints stuff on the telemetry (driver hub))
                telemetry.addData("x", drive.pose.position.x);
                telemetry.addData("y", drive.pose.position.y);
                telemetry.addData("heading (deg)", Math.toDegrees(drive.pose.heading.toDouble()));
                telemetry.addData("armCurrentPosition", up.getCurrentPosition());
                //telemetry.addData("outCurrentPosition", out.getCurrentPosition());
                telemetry.addData("clawCurrentPosition", servo_CLAW_position);
                telemetry.addData("manualOutControl", manualOutControl);
                telemetry.addData("up_true_target_pos", up_true_target_pos);
                telemetry.addData("up_current_pos", up.getCurrentPosition());
                telemetry.update();

                //idk what this does, something for ftc dashboard i think
                TelemetryPacket packet = new TelemetryPacket();
                packet.fieldOverlay().setStroke("#3F51B5");
                Drawing.drawRobot(packet.fieldOverlay(), drive.pose);
                FtcDashboard.getInstance().sendTelemetryPacket(packet);

                //increment the last_time
                last_time = runtime.seconds();
            }
        } else {
            throw new RuntimeException();
        }
    }
}
