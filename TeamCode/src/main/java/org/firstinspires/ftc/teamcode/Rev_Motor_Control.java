package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.CRServo;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.Servo;

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


@TeleOp(name="Rev_Motor_Control")
public class Rev_Motor_Control extends LinearOpMode {
// Front is on expansion hub port 3
    // viper slide is on expansion hub port 2
    // outake rotator on control hub 0
    // outtake claw on control hub 1
    // Declare motor and game pad
    // back left is on ctonrl hub 2
    // front left is on congtrol hub 1
    //
    private DcMotorEx revMotor;
    private CRServo servo_Intake; // Rev motor to control

    private CRServo servo_Outtake;
    private CRServo servo_Outtake_wrist;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry()); // sets up tel em

        //setup arm to use velocity
        revMotor = hardwareMap.get(DcMotorEx.class, "revMotor");
        revMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        revMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        revMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        servo_Outtake = hardwareMap.get(CRServo.class, "outtake"); //control hub 1
        servo_Outtake_wrist = hardwareMap.get(CRServo.class, "outtakeWrist");



        // Wait for the game to start
        waitForStart();

        // Controls
        while (opModeIsActive()) {
            if (gamepad1.a) {
                //use velocity mode to move so it doesn't we all funky with the smoothing of position mode
                revMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                revMotor.setVelocity(1000);
                 telemetry.addData("A down", true);
            } else if (gamepad1.y) {
                revMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                revMotor.setVelocity(-1000);
                telemetry.addData("Y down", true);
            } else {
                revMotor.setVelocity(0);
                telemetry.addData("Nothing pressed", true);
            }
            // servo one
            if (gamepad2.right_trigger > 0.8/* && servo_CLAW_position < 1000000000*/) {
                    servo_Outtake.setPower(1);
            } else if (gamepad2.right_bumper/* && servo_CLAW_position > -100000000*/) {
                    servo_Outtake.setPower(-1);
            } else {
                    servo_Outtake.setPower(0);
            }

            if (gamepad2.left_trigger > 0.8) {
                servo_Outtake_wrist.setPower(1);
            } else if (gamepad2.left_bumper/* && servo_CLAW_position > -100000000*/) { //TODO: these limits too.
                servo_Outtake_wrist.setPower(-1);
            } else {
                servo_Outtake_wrist.setPower(0);
            }

            // Telemetry to display motor status on the driver station
            //telemetry stuff (prints stuff on the telemetry (driver hub))
            telemetry.addData("armCurrentPosition", revMotor.getCurrentPosition());
            telemetry.addData("up_current_pos", revMotor.getCurrentPosition());
            telemetry.update();

            //idk what this does, something for ftc dashboard i think
            TelemetryPacket packet = new TelemetryPacket();
            packet.fieldOverlay().setStroke("#3F51B5");
            FtcDashboard.getInstance().sendTelemetryPacket(packet);


        }
    }
}

