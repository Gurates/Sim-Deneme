package rsim2.collision;

import rsim2.scene.SceneNode;

import java.util.Set;

public record GraspContact(SceneNode object, SceneNode tool, Set<SceneNode> touchLinks, float tolerance) {
    public GraspContact {
        touchLinks = Set.copyOf(touchLinks);
        if (object == null || object.isRobotPart() || tool == null || !tool.isRobotPart()
                || touchLinks.isEmpty() || !Float.isFinite(tolerance) || tolerance <= 0) {
            throw new IllegalArgumentException("Invalid grasp contact.");
        }
        for (SceneNode link : touchLinks) {
            if (!isToolLink(tool, link)) {
                throw new IllegalArgumentException("Touch link must belong to the selected tool: " + link.getId());
            }
        }
    }

    public boolean matches(SceneNode a, SceneNode b) {
        return a != null && b != null && (matchesOrder(a, b) || matchesOrder(b, a));
    }

    private boolean matchesOrder(SceneNode link, SceneNode part) {
        if (!touchLinks.contains(link) || !isToolLink(tool, link)) return false;
        for (SceneNode current = part; current != null; current = current.getParent()) {
            if (current == object) return !object.isRobotPart();
        }
        return false;
    }

    private static boolean isToolLink(SceneNode tool, SceneNode link) {
        for (SceneNode current = link; current != null && current.isRobotPart(); current = current.getParent()) {
            if (current == tool) return true;
        }
        return false;
    }
}
