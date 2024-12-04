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
@TeleOp(name="UPDATE_MAIN_Opmode")

public class Updated_Opmode_Main extends LinearOpMode {
    //setup arm variable
    private DcMotorEx misumi_slide_Motor;
    private DcMotorEx viper_slide_Moter;
    private CRServo servo_CLAW;
    private TouchSensor up_zero;
    //private boolean claw_closed = false;

    int arm_upper_lim = 6000;
    double servo_CLAW_power = 0.0;
    double servo_CLAW_position = 0.0;
    double manualOutControl = 0;
    int up_true_target_pos;

    //time stuff
    double last_time = 0;
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        //setup arm to use velocity
        misumi_slide_Motor = hardwareMap.get(DcMotorEx.class, "Misumi_Slide_Motor");
        misumi_slide_Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        misumi_slide_Motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        misumi_slide_Motor.setDirection(DcMotorSimple.Direction.REVERSE);

        //example position setup
        /*out = hardwareMap.get(DcMotorEx.class, "out");
        out.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        out.setTargetPosition(0);
        out.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        out.setDirection(DcMotorSimple.Direction.REVERSE);
        */
        //example velocity setup
        viper_slide_Moter = hardwareMap.get(DcMotorEx.class, "Viper_Slide_Motor");
        viper_slide_Moter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        viper_slide_Moter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        viper_slide_Moter.setDirection(DcMotorSimple.Direction.REVERSE);

        // servo_CLAW = hardwareMap.get(CRServo.class, "claw");

        //initilize touch sensor
        // up_zero = hardwareMap.get(TouchSensor.class, "up_zero");


        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
            MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

            waitForStart();

            while (opModeIsActive()) {


                //driving code taken from LocalizationTest
                /*
                Drive Controls
                Gamepad 1 :
                Left stick Controls all directional
                Right Stick Controls all Turns
                 */
                drive.setDrivePowers(new PoseVelocity2d(
                        new Vector2d(
                                -gamepad1.left_stick_y * 0.4,
                                -gamepad1.left_stick_x * 0.4
                        ),
                        -gamepad1.right_stick_x * 0.4
                ));

                drive.updatePoseEstimate();

                //arm code
                /*
                Misumi Controls
                Gamepad 2
                Button A slides misumi slide out
                Button B slides misumi slide in
                */

                if (gamepad1.a) {
                        //use velocity mode to move so it doesn't we all funky with the smoothing of position mode
                        misumi_slide_Motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        misumi_slide_Motor.setVelocity(1000);
                        telemetry.addData("Misumi Slide : Out", true);
                } else if (gamepad1.y) {
                        misumi_slide_Motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        misumi_slide_Motor.setVelocity(-1000);
                        telemetry.addData("Misumi Slide : In", true);
                } else {
                        misumi_slide_Motor.setVelocity(0);
                        telemetry.addData("Misumi Slide : Stationary", true);
                }

                /*
                Viper Slide Controls
                Gamepad 2
                Controlled via the D pad, the arrow thing
                Dpad_up slides viper slide up
                Dpad down slides viper slide down

                 */
                if (/*up.getCurrentPosition() < arm_upper_lim && */gamepad2.dpad_up) {
                    //use velocity mode to move so it doesn't we all funky with the smoothing of position mode
                    viper_slide_Moter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    viper_slide_Moter.setVelocity(1000);
                    up_true_target_pos = 0;
                } else if (!up_zero.isPressed() && gamepad2.dpad_down) {
                    viper_slide_Moter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    viper_slide_Moter.setVelocity(-1000);
                    up_true_target_pos = 0;
                } else {
                    viper_slide_Moter.setPower(500);
                    //use positon mode to stay up, as otherwise it would fall down. do some fancy stuff with up_true_target_pos to avoid the issue of it very slightly falling every tick /
                    // still makes no sense LMAO
                    if (up_true_target_pos == 0) {
                        viper_slide_Moter.setTargetPosition(viper_slide_Moter.getCurrentPosition());
                        up_true_target_pos = viper_slide_Moter.getCurrentPosition();
                    }
                    viper_slide_Moter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                }

                //make sure the upper and lower limits are actually at the upper and lower limits
                if (viper_slide_Moter.getCurrentPosition() < 0) {
                    viper_slide_Moter.setTargetPosition(0);
                } else if (viper_slide_Moter.getCurrentPosition() > arm_upper_lim) {
                    viper_slide_Moter.setTargetPosition(arm_upper_lim);
                }

                //manual out control
               // if (gamepad2.left_bumper) {
                    manualOutControl += -1000 * (runtime.seconds() - last_time); //TODO: make this -=
               // } else if ((gamepad2.left_trigger > 0.8f)) {
                    manualOutControl += 1000 * (runtime.seconds() - last_time);
               // }

                //reset manual out control
                //if (gamepad2.a) {
                //    manualOutControl = 0;
                //}

              //  out.setVelocity(500);
                // Keep the current expression of arm*2
                // add onto the expression: + myVariable
                // Increment myVariable so u have manual control
                // To "return" back to normal business set it to 0
               // out.setTargetPosition(((int) ((up.getCurrentPosition() * 1 / 2.0640625) + manualOutControl)));


                // This is servo code
                // Gamepad2.right_trigger is analog, so we need a compatative statment to use it as a digital button.
             //  if (gamepad2.right_trigger > 0.8/* && servo_CLAW_position < 1000000000*/) { //TODO: find a better solution for this limits so we can actually use them
              //      servo_CLAW_power = 1;
                   //servo_CLAW_position += 1 * (runtime.seconds() - last_time);
              //  } else if (gamepad2.right_bumper/* && servo_CLAW_position > -100000000*/) { //TODO: these limits too.
              //      servo_CLAW_power = -1;
                    //servo_CLAW_position += -1 * (runtime.seconds() - last_time);
             //   } else {
             //       servo_CLAW_power = 0;
            //    }

                // servo_CLAW.setPower(servo_CLAW_power);

                //NEW CLAW CODE WITH POSITION MODE!
                /*if (gamepad2.right_bumper) {
                    if (claw_closed) {
                        claw_closed = false;
                    } else if (!claw_closed) {
                        claw_closed = true;
                    }
                }
                if (claw_closed) {
                    servo_CLAW.setPosition(0.3703703704);
                } else {
                    servo_CLAW.setPosition(0);
                }
                servo_CLAW.getPosition()*/

                //telemetry stuff (prints stuff on the telemetry (driver hub))
                telemetry.addData("x", drive.pose.position.x);
                telemetry.addData("y", drive.pose.position.y);
                telemetry.addData("heading (deg)", Math.toDegrees(drive.pose.heading.toDouble()));
                telemetry.addData("MisumiCurrentPosition", misumi_slide_Motor.getCurrentPosition());
                telemetry.addData("ViperCurrentPosition", viper_slide_Moter.getCurrentPosition());
                telemetry.addData("manualOutControl", manualOutControl);
                telemetry.addData("up_true_target_pos", up_true_target_pos);
                telemetry.addData("Misumi_current_pos", misumi_slide_Motor.getCurrentPosition());
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
