package org.firstinspires.ftc.teamcode.Matei;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.sun.javafx.scene.traversal.Direction;
import org.firstinspires.ftc.robotcore.external.Telemetry;

@Autonomous(name = "aaa", group = "aa")
public class Main extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        // INITIALIZARE
        RobotComponents robotComponents = new RobotComponents();

        String[] mname = {"front_left_motor", "front_right_motor", "back_left_motor", "back_right_motor"};
        DcMotor[] motors = new DcMotor[4];
        String[] labels = {"pic", "pic", "pic", "pic"};
        DcMotor.Direction[] orientation = {DcMotor.Direction.REVERSE, DcMotor.Direction.FORWARD, DcMotor.Direction.REVERSE, DcMotor.Direction.FORWARD};

        if(!robotComponents.initDcMotors(hardwareMap, mname, motors, labels, orientation)) {
            telemetry.addData("plm", false);
            telemetry.update();
        }

        // RUN
        RobotActions robotActions = new RobotActions(robotComponents);
        robotActions.go("pic", new double[]{1, 1, 1, 1});
        sleep(2000);

    }
}
