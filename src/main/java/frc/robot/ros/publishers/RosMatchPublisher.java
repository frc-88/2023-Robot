package frc.robot.ros.publishers;

import org.ros.node.ConnectedNode;

import frc.robot.ros.messages.FrcMatch;
import tj2_interfaces.Match;

public class RosMatchPublisher extends RosSingleTopicPublisher<FrcMatch, Match> {

    public RosMatchPublisher(ConnectedNode connectedNode) {
        super(connectedNode, "/tj2/match", Match._TYPE);
    }

    @Override
    protected Match convert(FrcMatch data) {
        Match msg = makeMsg();
        msg.setMatchTime(data.match_time);
        msg.setTeamColor(data.team_color);
        msg.setTeamPosition((byte)data.team_position);
        msg.getMatchPeriod().setType(data.match_period.value);
        return msg;
    }
}
