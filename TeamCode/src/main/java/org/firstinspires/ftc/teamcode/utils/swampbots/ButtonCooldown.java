package org.firstinspires.ftc.teamcode.utils.swampbots;

public class ButtonCooldown {

    private long cooldown = 75L; // 75 milliseconds
    private long snapshot = 0L;

    public void updateSnapshot(long snapshotMilli) {
        this.snapshot = snapshotMilli;
    }
    public void updateSnapshot(double snapshotSec) {this.snapshot = (long) (snapshotSec * 1000);}

    public boolean ready(long runtime) {
        return(runtime - snapshot) > cooldown;
    }

    public void setCooldown(double seconds) {
        this.cooldown = (long) (seconds * 1000);
    }

    public long getCooldown () {
        return this.cooldown;
    }

    public long getSnapshot () {
        return this.snapshot;
    }
}
