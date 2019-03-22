package frc.team1071.lib;

import java.util.function.Consumer;

import edu.wpi.first.wpilibj.Notifier;

public class Task
{
    private final Notifier notifier;
    Consumer<Void> loopFunction;

    private final Runnable runnable = new Runnable() 
    {
        @Override
        public void run()
        {
            loopFunction.accept(null);
        }
    };

    //SafeVarargs
    public Task(double period, Consumer<Void> function)
    {
        notifier = new Notifier(runnable);
        loopFunction = function;
        notifier.startPeriodic(period);
    }
}