package ewbik.ik;

import java.util.Stack;

public final class PerfTimer {

    static Stack<Long> startTimes = new Stack<>();

    public static void start() {
        startTimes.push(System.nanoTime());
    }

}
