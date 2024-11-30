package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;


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

@TeleOp(name="Rev Motor Control")
public class Spool_Test extends LinearOpMode {

    // Declare motor and gamepad
    private DcMotorEx revMotor;  // Motor to control
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        //setup arm to use velocity
        revMotor = hardwareMap.get(DcMotorEx.class, "revMotor");
        revMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        revMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        revMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        // Initialize the hardware (motor and gamepad)

        // Initialize the gamepad (it is automatically available, but we just ensure it's referenced here)

        // Wait for the game to start
        waitForStart();

        // Loop to control the motor based on gamepad input
        while (opModeIsActive()) {
            if (gamepad1.a) {
                //use velocity mode to move so it doesn't we all funky with the smoothing of position mode
                revMotor.setVelocity(100);
                 telemetry.addData("A down", true);
            } else if (gamepad1.y) {
                revMotor.setVelocity(-10);
                telemetry.addData("Y down", true);
            } else {
                revMotor.setVelocity(0);
                telemetry.addData("Nothing pressed", true);
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

            //increment the last_time

        }
    }
}

