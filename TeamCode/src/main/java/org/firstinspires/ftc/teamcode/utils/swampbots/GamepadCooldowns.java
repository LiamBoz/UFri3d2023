package org.firstinspires.ftc.teamcode.utils.swampbots;

import java.util.LinkedList;
import java.util.List;

public class GamepadCooldowns {

    public ButtonCooldown dpUp     = new ButtonCooldown();
    public ButtonCooldown dpDown   = new ButtonCooldown();
    public ButtonCooldown dpLeft   = new ButtonCooldown();
    public ButtonCooldown dpRight  = new ButtonCooldown();

    public ButtonCooldown a    = new ButtonCooldown();
    public ButtonCooldown b    = new ButtonCooldown();
    public ButtonCooldown x    = new ButtonCooldown();
    public ButtonCooldown y    = new ButtonCooldown();

    public ButtonCooldown lb   = new ButtonCooldown();
    public ButtonCooldown rb   = new ButtonCooldown();
    public ButtonCooldown lt   = new ButtonCooldown();
    public ButtonCooldown rt   = new ButtonCooldown();

    public ButtonCooldown lStickB = new ButtonCooldown();
    public ButtonCooldown rStickB = new ButtonCooldown();

    List<ButtonCooldown> buttonCooldowns = new LinkedList<>();

    public GamepadCooldowns() {
        buttonCooldowns.add(dpUp);
        buttonCooldowns.add(dpDown);
        buttonCooldowns.add(dpLeft);
        buttonCooldowns.add(dpRight);

        buttonCooldowns.add(a);
        buttonCooldowns.add(b);
        buttonCooldowns.add(x);
        buttonCooldowns.add(y);

        buttonCooldowns.add(lb);
        buttonCooldowns.add(rb);
        buttonCooldowns.add(lt);
        buttonCooldowns.add(rt);

        buttonCooldowns.add(lStickB);
        buttonCooldowns.add(rStickB);
    }

    public void setCooldown(double cooldown) {
        for(ButtonCooldown button : buttonCooldowns) button.setCooldown(cooldown);
    }




}

