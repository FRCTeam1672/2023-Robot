package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Node.Height;
import frc.robot.Node.Translation;

public class Targeter {
    private Node targetNode;
    private Grid targetGrid;

    public Targeter() {
        setTarget(Grid.COOP, Nodes.HIGH_RIGHT);
    }

    public void setTarget(Grid grid, Node node) {
        if(grid != null) targetGrid = grid;
        if(node != null) targetNode = node;

        SmartDashboard.putString("Target Node", targetNode.toString());
    }
    public Command target(Grid grid, Node node) {
        return Commands.runOnce(() -> setTarget(grid, node));
    }

    public Node getTargetNode() {
        return targetNode;
    }
    public Grid getTargetGrid() {
        return targetGrid;
    }

    public static final class Nodes {
        public static final Node HIGH_LEFT = new Node(Height.HIGH, Translation.LEFT);
        public static final Node HIGH_CENTER = new Node(Height.HIGH, Translation.CENTER);
        public static final Node HIGH_RIGHT = new Node(Height.HIGH, Translation.RIGHT);
        public static final Node MID_LEFT = new Node(Height.MID, Translation.LEFT);
        public static final Node MID_CENTER = new Node(Height.MID, Translation.CENTER);
        public static final Node MID_RIGHT = new Node(Height.MID, Translation.RIGHT);
        public static final Node HYBRID_LEFT = new Node(Height.HYBRID, Translation.LEFT);
        public static final Node HYBRID_CENTER = new Node(Height.HYBRID, Translation.CENTER);
        public static final Node HYBRID_RIGHT = new Node(Height.HYBRID, Translation.RIGHT);
    }

    public enum Grid { LEFT, COOP, RIGHT }
}
