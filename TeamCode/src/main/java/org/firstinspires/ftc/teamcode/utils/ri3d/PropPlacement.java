package org.firstinspires.ftc.teamcode.utils.ri3d;

public enum PropPlacement {
    LEFT,
    CENTER,
    RIGHT,
    UNKNOWN;

    @Override
    public String toString() {
        switch (this) {
            case LEFT:    return "LEFT";
            case CENTER:      return "CENTER";
            case RIGHT:    return "RIGHT";
            case UNKNOWN:       return "UNKNOWN";
            default:            return "[Defaulted]";
        }
    }
}
