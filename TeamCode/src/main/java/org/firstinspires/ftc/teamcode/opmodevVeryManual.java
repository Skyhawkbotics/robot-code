package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.tuning.TuningOpModes;


//built from LocalizationTest, but adding the arm stuff
@TeleOp(name="opmodeVeryManual")
public class opmodevVeryManual extends LinearOpMode {

    //setup arm variable
    private DcMotorEx arm;
    private DcMotorEx out;
    private CRServo servo_CLAW;

    int arm_upper_lim = 1000000;
    double servo_CLAW_power = 0.0;
    double servo_CLAW_position = 0.0;

    //time stuff
    double last_time = 0;
    private ElapsedTime     runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        arm = hardwareMap.get(DcMotorEx.class, "arm");
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        out = hardwareMap.get(DcMotorEx.class, "out");
        out.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        out.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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
                if (arm.getCurrentPosition() < arm_upper_lim && gamepad1.dpad_up) {
                    arm.setVelocity(1000);
                }
                else if (arm.getCurrentPosition() > -1000000000 && gamepad1.dpad_down) {
                    arm.setVelocity(-1000);
                }
                else {
                    arm.setVelocity(0);
                }

                //for out
                if (out.getCurrentPosition() > -20000 && gamepad1.left_trigger > 0.8f) {
                    out.setVelocity(-500);
                }
                else if (out.getCurrentPosition() < 20000 && gamepad1.left_bumper) {
                    out.setVelocity(500);
                }
                else {
                    out.setVelocity(0);
                }


                //for claw
                if (gamepad1.right_bumper) { //TODO: find a better solution for this limits so we can actually use them
                    servo_CLAW_power = 1;
                    servo_CLAW_position += 1 * (runtime.seconds() - last_time);
                } else if (gamepad1.right_trigger > 0.8f) { //TODO: these limits too.
                    servo_CLAW_power = -1;
                    servo_CLAW_position += -1 * (runtime.seconds() - last_time);
                } else {
                    servo_CLAW_power = 0;
                }

                servo_CLAW.setPower(servo_CLAW_power);



                    //telemetry stuff and stuff for ftcDashboard
                telemetry.addData("x", drive.pose.position.x);
                telemetry.addData("y", drive.pose.position.y);
                telemetry.addData("heading (deg)", Math.toDegrees(drive.pose.heading.toDouble()));
                telemetry.addData("armCurrentPosition", arm.getCurrentPosition());
                telemetry.addData("outCurrentPosition", out.getCurrentPosition());
                telemetry.addData("clawCurrentPosition", servo_CLAW_position);
                telemetry.update();

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
