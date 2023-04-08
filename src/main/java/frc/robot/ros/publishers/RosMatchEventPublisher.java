package frc.robot.ros.publishers;

import org.ros.node.ConnectedNode;

import frc.robot.ros.messages.FrcMatchPeriod;
import tj2_interfaces.MatchPeriod;

public class RosMatchEventPublisher extends RosSingleTopicPublisher<FrcMatchPeriod, MatchPeriod> {
    public RosMatchEventPublisher(ConnectedNode connectedNode) {
        super(connectedNode, "/tj2/match_event", MatchPeriod._TYPE);
    }

    @Override
    protected MatchPeriod convert(FrcMatchPeriod data) {
        MatchPeriod msg = makeMsg();
        msg.setType(data.value);
        return msg;
    }
}
