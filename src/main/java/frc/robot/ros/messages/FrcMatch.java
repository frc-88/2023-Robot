package frc.robot.ros.messages;

public class FrcMatch {
    public double match_time = 0.0;
    public String team_color = "";
    public int team_position = 0;
    public FrcMatchPeriod match_period = FrcMatchPeriod.DISABLED;

    public FrcMatch(double match_time, String team_color, int team_position, FrcMatchPeriod match_phase)
    {
        this.match_time = match_time;
        this.team_color = team_color;
        this.team_position = team_position;
        this.match_period = match_phase;
    }
}
