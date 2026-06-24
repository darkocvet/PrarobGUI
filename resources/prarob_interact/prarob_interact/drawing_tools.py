"""
drawing_tools.py
----------------
The one high-level tool the LLM is allowed to call: `connect_and_avoid`.

DESIGN PHILOSOPHY
-----------------
Do NOT let the LLM emit individual joint angles or low-level moves -- that is
slow and unreliable for a timed drawing task. Instead the LLM only decides
*what* to draw (which COCO classes to connect / avoid) and calls this single
tool. This tool does the deterministic work:

    1. ask vision (YOLO) where those objects are       -> ctx.get_detections()
    2. convert pixel coordinates to robot base mm       -> ctx.image_to_base()
    3. plan a line that connects them and dodges the    -> plan_path()
       avoided objects' bounding boxes
    4. execute the path with the marker down            -> ctx.draw_polyline()

Steps 1, 2 and 4 belong to your teammates (vision / calibration / kinematics).
They are reached only through the small `RobotContext` adapter below, so you can
develop and test your LLM part independently and plug their real functions in
later. Each adapter method has a clearly marked TODO and a usable fallback.
"""

from typing import List, Dict, Any, Tuple, Optional
from langchain.agents import tool

# Module-level handle to the live robot context, set once by the agent node.
CTX: Optional["RobotContext"] = None


def set_context(ctx: "RobotContext") -> None:
    """Called once by nl_agent_node at startup so the tool can reach the robot."""
    global CTX
    CTX = ctx


# ===========================================================================
#  RobotContext: the seam between YOUR llm code and your TEAMMATES' code
# ===========================================================================
class RobotContext:
    """Adapter that the drawing tool uses to talk to the rest of the system.

    Wire each method to your teammates' real implementation. Until then, the
    fallbacks let the whole pipeline run (e.g. in RViz / simulation)."""

    def __init__(self, node):
        self.node = node  # rclpy Node, used for logging + status publishing

    # ---- status feedback to the GUI ---------------------------------------
    def status(self, text: str) -> None:
        """Publish a human-readable progress line to the GUI status topic."""
        self.node.publish_status(text)
        self.node.get_logger().info(text)

    # ---- VISION (teammate: robot vision / 2.3.1) --------------------------
    def get_detections(self, class_names: List[str]) -> List[Dict[str, Any]]:
        """Return detections for the requested classes.

        Expected format per detection:
            {"class": "car", "cx": <px>, "cy": <px>,
             "bbox": (x_min, y_min, x_max, y_max)}  # all in image pixels

        TODO: replace with your teammate's get_yolo_boxes tool / YOLO topic read.
        """
        return self.node.get_yolo_detections(class_names)

    # ---- CALIBRATION (teammate: camera<->base, 2.1.3) ---------------------
    def image_to_base(self, cx: float, cy: float) -> Tuple[float, float]:
        """Map an image pixel (cx, cy) to robot-base XY in millimetres.

        TODO: replace with your teammate's calibrated homography / transform.
        Fallback below is a naive linear map (ONLY for dry runs)."""
        return self.node.image_to_base(cx, cy)

    # ---- KINEMATICS / EXECUTION (teammate: IK + move_to_pose, 2.1.5) ------
    def draw_polyline(self, points_mm: List[Tuple[float, float]]) -> None:
        """Move the marker tip along the given XY polyline (mm) with the pen down.

        TODO: replace with calls to your move_to_pose / IK execution. Fallback
        just logs the waypoints."""
        self.node.draw_polyline(points_mm)


# ===========================================================================
#  Geometry: connect the points, dodge the obstacles  (this is YOUR planner;
#  if a teammate owns "tool path planning / 2.3.2", call theirs instead)
# ===========================================================================
import numpy as np
import heapq

A_STAR_RESOLUTION = 5.0 # mm
X_MIN_MM = 0.0
X_MAX_MM = 550.0
Y_MIN_MM = -350.0
Y_MAX_MM = 350.0

ROWS = int((X_MAX_MM - X_MIN_MM) / A_STAR_RESOLUTION)
COLS = int((Y_MAX_MM - Y_MIN_MM) / A_STAR_RESOLUTION)

def mm_to_grid(x_mm: float, y_mm: float) -> Tuple[int, int]:
    r = int((x_mm - X_MIN_MM) / A_STAR_RESOLUTION)
    c = int((y_mm - Y_MIN_MM) / A_STAR_RESOLUTION)
    r = max(0, min(ROWS - 1, r))
    c = max(0, min(COLS - 1, c))
    return r, c

def grid_to_mm(r: int, c: int) -> Tuple[float, float]:
    x_mm = X_MIN_MM + r * A_STAR_RESOLUTION
    y_mm = Y_MIN_MM + c * A_STAR_RESOLUTION
    return x_mm, y_mm

def astar(grid, start, goal):
    def heuristika(a, b): return np.sqrt((b[0]-a[0])**2 + (b[1]-a[1])**2)
    neighbors = [(0,1), (0,-1), (1,0), (-1,0), (1,1), (1,-1), (-1,1), (-1,-1)]
    close_set = set()
    came_from = {}
    gscore = {start: 0}
    fscore = {start: heuristika(start, goal)}
    oheap = []
    heapq.heappush(oheap, (fscore[start], start))
    
    while oheap:
        current = heapq.heappop(oheap)[1]
        if current == goal:
            data = []
            while current in came_from:
                data.append(current)
                current = came_from[current]
            data.append(start)
            return data[::-1]
            
        close_set.add(current)
        for i, j in neighbors:
            neighbor = current[0] + i, current[1] + j
            if 0 <= neighbor[0] < ROWS and 0 <= neighbor[1] < COLS:
                if grid[neighbor[0]][neighbor[1]] == 1: continue
            else: continue
                
            tentative_g_score = gscore[current] + heuristika(current, neighbor)
            if neighbor in close_set and tentative_g_score >= gscore.get(neighbor, 0): continue
            
            if tentative_g_score < gscore.get(neighbor, float('inf')):
                came_from[neighbor] = current
                gscore[neighbor] = tentative_g_score
                fscore[neighbor] = tentative_g_score + heuristika(neighbor, goal)
                heapq.heappush(oheap, (fscore[neighbor], neighbor))
    return False

def build_grid(obstacles_mm, margin_mm=40.0):
    grid = np.zeros((ROWS, COLS), dtype=int)
    for (x_min, y_min, x_max, y_max) in obstacles_mm:
        x_min_m = x_min - margin_mm
        y_min_m = y_min - margin_mm
        x_max_m = x_max + margin_mm
        y_max_m = y_max + margin_mm
        
        r_min, c_min = mm_to_grid(x_min_m, y_min_m)
        r_max, c_max = mm_to_grid(x_max_m, y_max_m)
        
        r_min, r_max = min(r_min, r_max), max(r_min, r_max)
        c_min, c_max = min(c_min, c_max), max(c_min, c_max)
        
        for r in range(r_min, r_max + 1):
            for c in range(c_min, c_max + 1):
                if 0 <= r < ROWS and 0 <= c < COLS:
                    grid[r][c] = 1
    return grid

def plan_path(points: List[Tuple[float, float]],
              obstacles: List[Tuple[float, float, float, float]]
              ) -> List[Tuple[float, float]]:
    """Build a polyline visiting `points` in order, dodging `obstacles` using A*."""
    if len(points) < 2:
        return list(points)

    grid = build_grid(obstacles)
    full_path_mm = []

    for i in range(len(points) - 1):
        start_mm = points[i]
        goal_mm = points[i+1]
        
        start_grid = mm_to_grid(start_mm[0], start_mm[1])
        goal_grid = mm_to_grid(goal_mm[0], goal_mm[1])
        
        # Free the start and goal cells and their immediate vicinity 
        # so we don't start/end in a blocked cell if it overlaps with an obstacle
        for r in range(start_grid[0]-1, start_grid[0]+2):
            for c in range(start_grid[1]-1, start_grid[1]+2):
                if 0 <= r < ROWS and 0 <= c < COLS: grid[r][c] = 0
        for r in range(goal_grid[0]-1, goal_grid[0]+2):
            for c in range(goal_grid[1]-1, goal_grid[1]+2):
                if 0 <= r < ROWS and 0 <= c < COLS: grid[r][c] = 0

        path_grid = astar(grid, start_grid, goal_grid)
        
        if not path_grid:
            if CTX: CTX.status(f"Warning: A* could not find path from {start_mm} to {goal_mm}. Using straight line.")
            if not full_path_mm:
                full_path_mm.append(start_mm)
            full_path_mm.append(goal_mm)
        else:
            path_mm = [grid_to_mm(r, c) for (r, c) in path_grid]
            if i == 0:
                full_path_mm.extend(path_mm)
            else:
                # skip first point to avoid duplicates
                full_path_mm.extend(path_mm[1:])
                
    return full_path_mm


# ===========================================================================
#  THE TOOL the LLM calls
# ===========================================================================
@tool
def connect_and_avoid(connect: List[str], avoid: List[str]) -> str:
    """Draw one continuous line that connects the given objects while avoiding others.

    Use this for every drawing command. Arguments MUST be exact YOLO/COCO class
    names (lowercase), e.g. connect=["airplane", "car"], avoid=["sports ball"].

    :param connect: ordered list of object classes to connect with a line (>=2).
    :param avoid:   list of object classes whose pictures the line must not cross.
    """
    if CTX is None:
        return "ERROR: robot context not initialised."
    if len(connect) < 2:
        return "ERROR: need at least two objects to connect."

    CTX.status(f"Command parsed: connect {connect}, avoid {avoid}.")

    # 1) Locate everything we care about.
    wanted = list(dict.fromkeys(connect + avoid))  # unique, keep order
    detections = CTX.get_detections(wanted)
    found = {d["class"]: d for d in detections}    # first hit per class

    missing = [c for c in connect if c not in found]
    if missing:
        return (f"Could not find these objects on the board: {missing}. "
                f"Detected: {list(found.keys())}.")

    CTX.status(f"Detected objects: {list(found.keys())}.")

    # 2) Centroids of objects-to-connect -> base frame (mm).
    connect_pts = [CTX.image_to_base(found[c]["cx"], found[c]["cy"]) for c in connect]

    # 3) Bounding boxes of objects-to-avoid -> base frame (mm).
    obstacles = []
    for c in avoid:
        if c in found:
            x0, y0, x1, y1 = found[c]["bbox"]
            bx0, by0 = CTX.image_to_base(x0, y0)
            bx1, by1 = CTX.image_to_base(x1, y1)
            obstacles.append((min(bx0, bx1), min(by0, by1),
                              max(bx0, bx1), max(by0, by1)))

    # 4) Plan and execute.
    path = plan_path(connect_pts, obstacles)
    CTX.status(f"Planned path with {len(path)} waypoints. Drawing...")
    CTX.draw_polyline(path)

    drawn = " -> ".join(connect)
    avoided = ", ".join(avoid) if avoid else "nothing"
    return (f"Done. Drew a line connecting {drawn} "
            f"({len(path)} waypoints), avoiding {avoided}.")


# Export the tools you want ROSA to have.
DRAWING_TOOLS = [connect_and_avoid]
