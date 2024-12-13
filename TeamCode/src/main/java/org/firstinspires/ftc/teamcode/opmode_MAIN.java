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
    int arm_upper_lim = 4350;
    double servo_CLAW_power = 0.0;
    double manualOutControl = 0;
    int up_true_target_pos;
    int out_true_target_pos;
    double servo_outtake_wrist_location = 0;
    double servo_intake_wrist_location = 0;




    //vars for set positions for transfer:
    ///TODO: CHANGE THESE
    int transfer_step = 0;
    double intake_wrist_pos_transfer = 0.1;
    double outtake_wrist_pos_transfer = 0.2;
    double out_pos_transfer = 30;
    double up_pos_transfer1 = 20;
    double up_pos_transfer2 = 10;
    double up_pos_transfer3 = 20;
    double outtake_wrist_pos_ready = 300;



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
                                -gamepad1.left_stick_y * 0.7,
                                -gamepad1.left_stick_x * 0.7
                        ),
                        -gamepad1.right_stick_x * 0.4
                ));

                drive.updatePoseEstimate();

                //arm code
                if (gamepad2.left_stick_y < -0.1) { //left stick -, is going up!
                    //use velocity mode to move so it doesn't we all funky with the smoothing of position mode
                    up.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    up.setVelocity(gamepad2.left_stick_y * -1200);
                    up_true_target_pos = 0;
                }
                else if (!up_zero.isPressed() && gamepad2.left_stick_y > 0.1) { //left stick +, going down
                    up.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    up.setVelocity(gamepad2.left_stick_y * -1200);
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

                if (gamepad2.right_stick_y > 0.1) {
                    //use velocity mode to move so it doesn't we all funky with the smoothing of position mode
                    out.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    out.setVelocity(gamepad2.right_stick_y * 500);
                    out_true_target_pos = 0;
                }
                else if (!out_zero.isPressed() && gamepad2.right_stick_y < -0.1) {
                    out.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    out.setVelocity(gamepad2.right_stick_y * 500);
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

                // manual outtake wrist location
                if (gamepad2.dpad_up) {
                    servo_outtake_wrist_location += 0.05;
                } if (gamepad2.dpad_down) {
                    servo_outtake_wrist_location -= 0.05;
                }

                if (servo_outtake_wrist_location > 1) {
                    servo_outtake_wrist_location = 1;
                } else if (servo_outtake_wrist_location < 0) {
                    servo_outtake_wrist_location = 0;
                }

                servo_outtake_wrist.setPosition(servo_outtake_wrist_location);

                // manual intake wrist location
                if (gamepad2.dpad_right) {
                    servo_intake_wrist_location += 0.05;
                } if (gamepad2.dpad_left) {
                    servo_intake_wrist_location -= 0.05;
                }

                if (servo_intake_wrist_location > 1) {
                    servo_intake_wrist_location = 1;
                } else if (servo_intake_wrist_location < 0) {
                    servo_intake_wrist_location = 0;
                }

                servo_intake_wrist.setPosition(servo_intake_wrist_location);


                //SIMPLE TRANSFER BUTTON (turns on both servos at once)
                if (gamepad2.a) {
                    servo_outtake.setPower(1);//TODO: CHANGE THIS IF ITS WRONG
                    servo_intake.setPower(-1);//TODO: CHANGE THIS IF ITS WRONG // got it trev
                }

                //SIMPLE TRANSFER SETUP BUTTON (sets wrists to the right value)
                if (gamepad2.b) {
                    servo_intake_wrist_location = intake_wrist_pos_transfer; //TODO: CHANGE THIS VAR ABOVE!
                    servo_outtake_wrist_location = outtake_wrist_pos_transfer; //TODO: CHANGE THIS VAR ABOVE!
                }

                //MACROS

                //auto-transfer
                if (gamepad2.y) {
                    if (transfer_step == 0) { //get in position
                        //1. get out to the position of out_transfer limit switch
                        if (!out_transfer.isPressed() && out.getCurrentPosition() > out_pos_transfer) {
                            out.setVelocity(-100);
                        } else if (!out_transfer.isPressed() && out.getCurrentPosition() < out_pos_transfer) {
                            out.setVelocity(100);
                        } else {
                            //do more fancy stuff to make it stay at the right value!
                            out.setVelocity(0);
                            out_true_target_pos = out.getCurrentPosition();
                            out.setTargetPosition(out.getCurrentPosition());
                            out.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        }
                        //2. set position of servo_intake_wrist
                        servo_intake_wrist.setPosition(intake_wrist_pos_transfer);
                        //3. put viper slide up a bit
                        if (up.getCurrentPosition() > up_pos_transfer1) {
                            up.setVelocity(-100);
                        } else if (up.getCurrentPosition() < up_pos_transfer1) {
                            up.setVelocity(100);
                        } else {
                            //do more fancy stuff to make it stay at the right value!
                            up.setVelocity(0);
                            up_true_target_pos = up.getCurrentPosition();
                            up.setTargetPosition(up.getCurrentPosition());
                            up.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        }
                        servo_outtake_wrist.setPosition(outtake_wrist_pos_transfer);
                    } else if (transfer_step == 1) { //put claws in positiion
                        //viper slide back down a bit, can't go to this pos at start because when rotating claw would bump into sample in other claw!
                        if (up.getCurrentPosition() > up_pos_transfer2) {
                            up.setVelocity(-50);
                        } else if (up.getCurrentPosition() < up_pos_transfer2) {
                            up.setVelocity(50);
                        } else {
                            //do more fancy stuff to make it stay at the right value!
                            up.setVelocity(0);
                            up_true_target_pos = up.getCurrentPosition();
                            up.setTargetPosition(up.getCurrentPosition());
                            up.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        }
                        servo_outtake.setPower(1); //set to recieve sample
                    } else if (transfer_step == 2) { //sample into other claw
                        servo_intake.setPower(-1); //spit out the sample!
                        if (up.getCurrentPosition() > up_pos_transfer3) {
                            up.setVelocity(-50);
                        } else if (up.getCurrentPosition() < up_pos_transfer3) {
                            up.setVelocity(50);
                        } else {
                            //do more fancy stuff to make it stay at the right value!
                            up.setVelocity(0);
                            up_true_target_pos = up.getCurrentPosition();
                            up.setTargetPosition(up.getCurrentPosition());
                            up.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        }
                    } else if (transfer_step == 3) { //
                        //zero out the out slide
                        if (!out_zero.isPressed() && out.getCurrentPosition() > 0) {
                            out.setVelocity(-100);
                        } else if (!out_zero.isPressed() && out.getCurrentPosition() < 0) {
                            out.setVelocity(100);
                        } else {
                            //do more fancy stuff to make it stay at the right value!
                            out.setVelocity(0);
                            out_true_target_pos = out.getCurrentPosition();
                            out.setTargetPosition(out.getCurrentPosition());
                            out.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        }
                        //get outtake in position outtake
                        servo_outtake_wrist.setPosition(outtake_wrist_pos_ready);
                    }

                    //moving on, to next step if everything is accomplished. If changing something in one of the steps, make sure to change the if statment!
                    if (out_transfer.isPressed() && up.getCurrentPosition() == up_pos_transfer1 && servo_intake_wrist.getPosition() == intake_wrist_pos_transfer && servo_outtake_wrist.getPosition() == out_pos_transfer && transfer_step < 1) {
                        transfer_step = 1;
                    } else if (transfer_step == 1 && up.getCurrentPosition() == up_pos_transfer2) {
                        transfer_step = 2;
                    } else if (transfer_step == 2 &&  up.getCurrentPosition() == up_pos_transfer3) {
                        transfer_step = 3;
                    }

                } else { //reset so it will work again
                    transfer_step = 0;
                }


                //telemetry stuff (prints stuff on the telemetry (driver hub))
                telemetry.addData("x", drive.pose.position.x);
                telemetry.addData("y", drive.pose.position.y);
                telemetry.addData("heading (deg)", Math.toDegrees(drive.pose.heading.toDouble()));

                telemetry.addData("out_current_pos", out.getCurrentPosition());
                telemetry.addData("out_true_target_pos", out_true_target_pos);
                telemetry.addData("up_true_target_pos", up_true_target_pos);
                telemetry.addData("up_current_pos", up.getCurrentPosition());
                telemetry.addData("intake_wrist_current_pos", servo_intake_wrist_location);
                telemetry.addData("outtake_wrist_current_pos", servo_outtake_wrist_location);
                telemetry.addData("gamepad2.left_stick_y", gamepad2.left_stick_y);



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
