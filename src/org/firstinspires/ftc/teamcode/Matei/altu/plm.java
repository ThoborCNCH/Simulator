package org.firstinspires.ftc.teamcode.Matei.altu;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "plm", group = "a")
public class plm extends Cabral {


    @Override
    public void runOpMode() {
        init(hardwareMap);
        waitForStart();
        // while(opModeIsActive()){
        // telemetry.addData("lf")
        // }
        if(opModeIsActive()){
//            runBitch(11, 10);
//            sleep(1000);
//            daTeCaMiBagPl(11, 10);
//            sleep(1000);
//            invartaTeAnPl(11, -90);
//            sleep(1000);
            mergiAnPulea(-0.1, 0.06, -0.2, 0.2,
                    -16, -32, 0, 46);

            // sleep(1000);
            // invartaTeAnPl(0.1, 45);
        }

// //        pct A:
//         runBitch(0.1, 50);
//         mergiAnPulea(-0.5, 0.3, -1, 1,
//                 -16, -32, 0, 46);
//         runBitch(0.1,-20);
//         invartaTeAnPl(0.1, -90);

//        pct B:
//        runBitch(1, 80);
//        daTeCaMiBagPl(1, -20);
//        runBitch(1, -20);

//        pct C:
//        runBitch(1, 120);
//        mergiAnPulea(-0.5, 0.3, -1, 1,
//                -16, -32, 0, 46);
//        runBitch(1,-20);
//        daTeCaMiBagPl(1, 80);
//        invartaTeAnPl(1, -90);

    }
}



// package org.firstinspires.ftc.teamcode.Matei.altu;

// import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
// import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
// import com.qualcomm.robotcore.hardware.DcMotor;

// @Autonomous(name = "MicunealtaSecreta1", group = "a")
// public class MicunealtaSecreta1 extends LinearOpMode {
//     DcMotor motor;
//     @Override
//     public void runOpMode() throws InterruptedException {
//         motor = hardwareMap.dcMotor.get("back_left_motor");
//         waitForStart();
//         motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//         motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//         motor.setTargetPosition(1000000);
//         motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//         motor.setPower(1);

//         while(opModeIsActive()){
//             telemetry.addData("asd", motor.getCurrentPosition());
//             telemetry.update();
//         }

//     }
// }





