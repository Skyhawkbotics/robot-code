package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Trajectory;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.TrajectoryBuilder;

import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.acmerobotics.roadrunner.*;

import java.lang.Math;

import kotlin.OverloadResolutionByLambdaReturnType;

@Autonomous(name = "autonomous_MAIN")
public class autonomous_MAIN extends LinearOpMode {
    public class Elevator {
        private TouchSensor up_zero;
        private DcMotorEx up;

        public Elevator(HardwareMap hardwareMap) {
            up_zero = hardwareMap.get(TouchSensor.class, "up_zero");
            up = hardwareMap.get(DcMotorEx.class, "up");
        }

        public class CalibrateDown implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    up.setVelocity(-500);
                    initialized = true;
                }
                packet.put("Button is pressed", String.valueOf(up_zero.isPressed()));
                if (!up_zero.isPressed()) {
                    return true;
                } else {
                    up.setVelocity(0);
                    return false;
                }
            }
        }

        /*public class ElevatorMove implements Action {

            private int pos = 0;
            public ElevatorMove(int pos_input) {
                pos = pos_input;
            }
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (up.getCurrentPosition() < pos) {
                    up_true_target_pos = 0;
                } else (up.getCurrentPosition() > pos) {
                    up.setVelocity();
                    up_true_target_pos = 0;
                }  else {
                    up.setTargetPosition(up.getCurrentPosition());
                    up_true_target_pos = up.getCurrentPosition();
                }
            }

        }*/

        public Action calibrateDown() {
            return new CalibrateDown();
        }

        /*public Action elevatorMove(int pos) {
            return new  ElevatorMove(pos);
        }*/
    }

    private DcMotorEx up; //name of motor is up(in the code);
    private DcMotorEx out;
    // DcMotorEx is any dc motor that uses the regular motor ports (on left side of control and expantion hub)
    private CRServo servo_CLAW;
// CRServo means its continuous rotation servo
    private TouchSensor up_zero;
    double servo_CLAW_power = 0.0;
    double servo_CLAW_position = 0.0;

    double last_time = 0;
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0.0,0.0,0.0));
        //initalize up using position mode basically just copied from somewhere
        up = hardwareMap.get(DcMotorEx.class, "up"/*this is the name of the motor on the acutal robot (changed through driver hub)*/);
        up.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        up.setMode(DcMotor.RunMode.RUN_USING_ENCODER); //using velocity mode!
        up.setDirection(DcMotorSimple.Direction.REVERSE);

        //initalize out
        //out = hardwareMap.get(DcMotorEx.class, "out");
        //out.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //out.setTargetPosition(0);
        //out.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //initialize claw servo
        servo_CLAW = hardwareMap.get(CRServo.class, "claw");

        //initialize touch sensor
        up_zero = hardwareMap.get(TouchSensor.class, "up_zero");

        Elevator elevator = new Elevator(hardwareMap);

        waitForStart(); // it might wait for start

        //put the out motor out
        //out.setVelocity(200);
        //out.setTargetPosition(-1088);

        //build the  trajectory leftCorner
        TrajectoryActionBuilder leftCorner = drive.actionBuilder(/*start position*/new Pose2d(0.0, 5.0, 0.0)) // tells the robot where it's going to start?

        //directions from start postion
            .strafeTo(new Vector2d(0.0, 10.0)) // went to the left 10 units(inches?)
            .waitSeconds(3.0) //waits for 3 seconds
            //.lineToX(10.0) //foward 10 units, seems useless as it kind of gets crooked, and not the right angle
            .strafeTo(new Vector2d(10, 15));
            //.turn(2) //turns way further than 2 radians, who knows why. real
            //    .setTangent(Math.toRadians(180));




        //main loop
        while(!opModeIsActive() && !isStopRequested()) {


        }
        waitForStart();
        if (isStopRequested()) return;
        //run the action leftCorner, first building it to make it runnable as a action, as a result we can no longer edit this action!
        Action leftCornerBuild;
        leftCornerBuild = leftCorner.build();
        Actions.runBlocking(
                new ParallelAction(
                        leftCornerBuild,
                        elevator.calibrateDown()
                )
        );

        //zeros out up using the sensor, (goes down until button is pressed)
        if (!up_zero.isPressed()) {
            up.setVelocity(-500);
        } else {
            up.setVelocity(0);
        }
    }
}


