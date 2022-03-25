//package org.firstinspires.ftc.teamcode.Matei.plm_nush;
//
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//
//@Autonomous(name = "AlbastruStanga", group = "a")
//public class AlbastruStanga extends LinearOpMode {
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        waitForStart();
//
//        if (opModeIsActive()) {
//            Dumnezeu iisus = new Dumnezeu();
//            iisus.init(hardwareMap);
//            iisus.mergi(1, 28);
//
//            Trasee traseu = new Trasee(iisus);
//
//            int caz = (int) (Math.random() * 3 + 1);
//
//            telemetry.update();
//
//            if (caz == 1) {
//                traseu.albastruMergiLaPunctulA();
//            } else if (caz == 2) {
//                traseu.albastruMergiLaPunctulB();
//            } else {
//                traseu.albastruMergiLaPunctulC();
//            }
//
//            iisus.setMotorsPower(0, 0, 0, 0);
//            while (opModeIsActive()) {
////                iisus.checkDirection();
//            }
//        }
//    }
//}
