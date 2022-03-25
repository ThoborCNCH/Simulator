package org.firstinspires.ftc.teamcode.Matei.hanga;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Autonomus_Maximus", group = "Romulus")
public class asdkjbasd extends  askdj{
    int gubildu = 12;
    double putere = 0;
    @Override
    public void runOpMode() {
        init(hardwareMap);
        waitForStart();

        if(gubildu == 1)
        {
            putere = 0.1;
        }
        else
        {
            putere = 1;
        }

        if(opModeIsActive()){
            runBitch(putere, 30);
            sleep(1000);
            // daTeCaMiBagPl(putere, 10);
            // sleep(1000);
            invartaTeAnPl(putere, 45);
            //rotire_poz_dr_neg_st(45);
            // sleep(1000);
            //mergiAnPulea(-0.1, 0.06, -0.2, 0.2, -16, -32, 0, 46);

        }

    }
}
