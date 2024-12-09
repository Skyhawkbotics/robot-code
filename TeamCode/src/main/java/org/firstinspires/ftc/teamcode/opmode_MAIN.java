package org.firstinspires.ftc.teamcode;

import android.text.method.Touch;

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
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.tuning.TuningOpModes;


//built from LocalizationTest, but adding the arm stuff
@TeleOp(name="MAIN_opmode")
public class opmode_MAIN extends LinearOpMode {

    //setup arm variable
    private DcMotorEx up;
    private DcMotorEx out;
    private CRServo servo_outtake;
    private CRServo servo_intake;
    private Servo servo_intake_wrist;
    private Servo servo_outtake_wrist;
    private TouchSensor up_zero;
    private TouchSensor out_zero;
    private TouchSensor out_transfer;
    int arm_upper_lim = 6000;
    double servo_CLAW_power = 0.0;
    double manualOutControl = 0;
    int up_true_target_pos;
    int out_true_target_pos;

    //time stuff
    double last_time = 0;
    private ElapsedTime     runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        

            //setup arm to use velocity
            up = hardwareMap.get(DcMotorEx.class, "up");
            up.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            up.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            up.setDirection(DcMotorSimple.Direction.REVERSE);

            //example position setup
            out = hardwareMap.get(DcMotorEx.class, "out");
            out.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            out.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            out.setDirection(DcMotorSimple.Direction.REVERSE);

            //example velocity setup
            //up = hardwareMap.get(DcMotorEx.class, "up");
            //up.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            //up.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            //up.setDirection(DcMotorSimple.Direction.REVERSE);

        //setup servos for intake and outtake
        servo_intake = hardwareMap.get(CRServo.class, "intake");
        servo_outtake = hardwareMap.get(CRServo.class, "outtake");
        servo_intake_wrist = hardwareMap.get(Servo.class, "intakeWrist");
        servo_outtake_wrist = hardwareMap.get(Servo.class, "outtakeWrist");



        //initilize touch sensor
        up_zero = hardwareMap.get(TouchSensor.class, "up_zero");
        out_zero = hardwareMap.get(TouchSensor.class, "out_zero");
        out_transfer = hardwareMap.get(TouchSensor.class, "out_transfer");


        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
            MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

            waitForStart();

            while (opModeIsActive()) {


                //driving code taken from LocalizationTest
                drive.setDrivePowers(new PoseVelocity2d(
                        new Vector2d(
                                -gamepad1.left_stick_y * 0.4,
                                -gamepad1.left_stick_x * 0.4
                        ),
                        -gamepad1.right_stick_x * 0.4
                ));

                drive.updatePoseEstimate();

                //arm code
                if (/*up.getCurrentPosition() < arm_upper_lim && */gamepad2.dpad_up) {
                    //use velocity mode to move so it doesn't we all funky with the smoothing of position mode
                    up.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    up.setVelocity(1000);
                    up_true_target_pos = 0;
                }
                else if (!up_zero.isPressed() && gamepad2.dpad_down) {
                    up.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    up.setVelocity(-1000);
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

                if (gamepad2.dpad_left) {
                    //use velocity mode to move so it doesn't we all funky with the smoothing of position mode
                    out.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    out.setVelocity(1000);
                    out_true_target_pos = 0;
                }
                else if (!out_zero.isPressed() && gamepad2.dpad_right) {
                    out.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    out.setVelocity(-1000);
                    out_true_target_pos = 0;
                } else {
                    out.setPower(500);
                    //use positon mode to stay up, as otherwise it would fall down. do some fancy stuff with up_true_target_pos to avoid the issue of it very slightly falling every tick
                    if (out_true_target_pos == 0) {
                        out.setTargetPosition(out.getCurrentPosition());
                        out_true_target_pos = out.getCurrentPosition();
                    }
                    out.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                }

                //make sure the upper and lower limits are actually at the upper and lower limits
                if (up.getCurrentPosition() < 0) {
                    up.setTargetPosition(0);
                } else if (up.getCurrentPosition() > arm_upper_lim) {
                    up.setTargetPosition(arm_upper_lim);
                }



                // Gamepad2.right_trigger is analog, so we need a compatative statment to use it as a digital button.
                //servo intake control
                if (gamepad2.right_trigger > 0.8/* && servo_CLAW_position < 1000000000*/) { //NO LONGER NEEDED: find a better solution for this limits so we can actually use them
                    servo_intake.setPower(1);
                } else if (gamepad2.right_bumper) { //NO LONGER NEEDED: these limits too.
                    servo_intake.setPower(-1);
                } else {
                    servo_intake.setPower(0);
                }

                //servo outtake control
                if (gamepad2.left_trigger > 0.8/* && servo_CLAW_position < 1000000000*/) { //NO LONGER NEEDED: find a better solution for this limits so we can actually use them
                    servo_outtake.setPower(1);
                } else if (gamepad2.left_bumper) { //NO LONGER NEEDED: these limits too.
                    servo_outtake.setPower(-1);
                } else {
                    servo_outtake.setPower(0);
                }


                //telemetry stuff (prints stuff on the telemetry (driver hub))
                telemetry.addData("x", drive.pose.position.x);
                telemetry.addData("y", drive.pose.position.y);
                telemetry.addData("heading (deg)", Math.toDegrees(drive.pose.heading.toDouble()));
                telemetry.addData("armCurrentPosition", up.getCurrentPosition());
                telemetry.addData("outCurrentPosition", out.getCurrentPosition());
                telemetry.addData("manualOutControl", manualOutControl);
                telemetry.addData("up_true_target_pos", up_true_target_pos);
                telemetry.addData("up_current_pos", up.getCurrentPosition());
                telemetry.addData("servo_claw_power", servo_CLAW_power);
                //4 telemetry.addData("clawCurrentPos", servo_CLAW.getPosition());
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
