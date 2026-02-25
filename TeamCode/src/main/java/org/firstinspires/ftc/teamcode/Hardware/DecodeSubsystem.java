package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Hardware.BaseHardware;
import org.firstinspires.ftc.teamcode.Hardware.Limey;
import org.firstinspires.ftc.teamcode.Hardware.DriveTrain;
import org.firstinspires.ftc.teamcode.Field.DecodeField;

public class DecodeSubsystem  {

    private Limey limey;
    private DriveTrain driveTrain;

    public HardwareMap hardwareMap = null;
    public Telemetry telemetry = null;

    public int currentTagID = -1;
    public double tagFieldX = Double.NaN;
    public double tagFieldY = Double.NaN;
    public double tagFieldHeadingDegree = Double.NaN;

    public double robotHeadingDeg = 0;

    @Override
    public void init (){

    }

    public void init_loop(){}

    public void start(){}


    public void setDependencies (Limey limey, DriveTrain drivetrain){

        this.limey = limey;
        this.driveTrain = drivetrain;
    }

    public void loop(){

        if(limey == null || driveTrain == null) return;

        currentTagID = limey.getTagID();
        robotHeadingDeg = driveTrain.getCurrentHeading();

        if(currentTagID == -1){
            tagFieldX = Double.NaN;
            tagFieldY = Double.NaN;
            tagFieldHeadingDegree = Double.NaN;
            return;
        }

        tagFieldX = DecodeField.getTAGSx(currentTagID);
        tagFieldY = DecodeField.getTAGSy(currentTagID);
        tagFieldHeadingDegree = DecodeField.getTAGSHeadingdegree(currentTagID);

        telemetry.addData("Robot Heading",robotHeadingDeg);
    }

    public void stop(){}


}
