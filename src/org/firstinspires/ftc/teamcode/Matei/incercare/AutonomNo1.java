package org.firstinspires.ftc.teamcode.Matei.incercare;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "AutonomNo1", group = "A")
public class AutonomNo1 extends LinearOpMode {

    SfPavel sfPavel = new SfPavel();
    Trasee trasee = new Trasee(sfPavel);

    @Override
    public void runOpMode() throws InterruptedException {
//        waitForStart();
        while (!opModeIsActive() && !isStopRequested()) {
            telemetry.addData("status", "Waiting for start command.....");
            telemetry.update();
        }


        if (opModeIsActive()) {
            sfPavel.init(hardwareMap);

            int random = (int) Math.floor(Math.random() * 3 + 1);


            sfPavel.mergi(1, 40);
            sfPavel.rotire(0.4, -90);
//            idle();
            //detect
            sfPavel.rotire(0.4, 90);

            if (random == 1) {
                System.out.println("1");
                trasee.albastruMergiLaPunctulA();
            } else if (random == 2) {
                System.out.println("2");
                trasee.albastruMergiLaPunctulB();
            } else {
                System.out.println("3");
                trasee.albastruMergiLaPunctulC();
            }

            sfPavel.setMotorsPower(0, 0, 0, 0);
//            while (opModeIsActive()) {
//            }
        }
    }
}