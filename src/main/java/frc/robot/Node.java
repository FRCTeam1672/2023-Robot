package frc.robot;

public class Node {
    private Height height;
    private Translation translation;

    Node(Height height, Translation translation) {
        this.height = height;
        this.translation = translation;
    }

    public Height getHeight() {
        return height;
    }
    public Translation getTranslation() {
        return translation;
    }

    public String toString() {
        return String.format("%s\n%s", height, translation);
    }

    public enum Height { HIGH, MID, HYBRID }
    public enum Translation { LEFT, CENTER, RIGHT }
}
