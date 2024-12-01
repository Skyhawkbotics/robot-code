package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name="Rev_Motor_Control")
public class Rev_Motor_Control extends LinearOpMode {

    // Declare motor and game pad
    private DcMotorEx revMotor;  // Rev motor to control
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry()); // sets up tel em

        //setup arm to use velocity
        revMotor = hardwareMap.get(DcMotorEx.class, "revMotor");
        revMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        revMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        revMotor.setDirection(DcMotorSimple.Direction.REVERSE);


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

