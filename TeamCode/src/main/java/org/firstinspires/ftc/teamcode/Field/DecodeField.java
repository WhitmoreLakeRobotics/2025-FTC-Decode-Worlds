package org.firstinspires.ftc.teamcode.Field;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Field.DecodeField;

import java.util.Map;

public class DecodeField {

    public static final int TAGS_20 = 20;
    public static final int TAGS_24 = 24;

    public static final double TAGS_20_x = 1.829;
    public static final double TAGS_20_y = 0.381;
    public static final double TAGS_20_Heading_degree = 90;

    public static final double TAGS_24_x = 1.829;
    public static final double TAGS_24_y = 3.2766;
    public static final double TAGS_24_Heading_degree = -90;

    public static final double getTAGSx(int id){
        if (id == TAGS_20_x) return TAGS_20_x;
        if (id == TAGS_24_x) return TAGS_24_x;
        return Double.NaN;
    }

    public static final double getTAGSy(int id) {
        if (id == TAGS_20) return TAGS_20_y;
        if (id == TAGS_24) return TAGS_24_y;
        return Double.NaN;
    }

    public static final double getTAGSHeadingdegree(int id) {
         if (id == TAGS_20) return TAGS_20_Heading_degree;
         if (id == TAGS_24) return TAGS_24_Heading_degree;
         return Double.NaN;
    }





}
