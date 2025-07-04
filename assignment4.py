#%%
from scipy.spatial import KDTree
import numpy as np
import math
import random
import vedo as vd
import time
from vedo.pyplot import DirectedGraph

vd.settings.default_backend = 'vtk'
#%%
class Sample:
    def __init__(self, x, parent = None, cost=0.0):
        self.x = x
        self.parent = parent
        self.cost   = cost

# find the nearest sample using KDTree
def NearestSample(samples, x):
    tree = KDTree([sample.x for sample in samples])
    dist, sid = tree.query(x)
    return sid


def bresenham(x1, y1, x2, y2):
    # (unchanged)
    dx, dy = abs(x2 - x1), abs(y2 - y1)
    x, y = x1, y1
    sx = 1 if x2>x1 else -1
    sy = 1 if y2>y1 else -1

    if dx > dy:
        err = dx/2
        while x != x2:
            yield x, y
            err -= dy
            if err < 0:
                y += sy
                err += dx
            x += sx
        yield x, y
    else:
        err = dy/2
        while y != y2:
            yield x, y
            err -= dx
            if err < 0:
                x += sx
                err += dy
            y += sy
        yield x, y

def extract_path(samples, goal_idx, dest):
    """
    Reconstructs the RRT path from source → goal → dest.
    - samples: list of Sample objects
    - goal_idx: index of the goal node in samples
    - dest: (x,y) tuple of the true destination
    Returns: list of (x,y) waypoints from source to dest.
    """
    path = [tuple(dest)]
    idx  = goal_idx
    while idx is not None:
        path.append(tuple(samples[idx].x))
        idx = samples[idx].parent
    return path[::-1]


def recolor_path_edges(path_pts, color='green', width=4):
    """
    Given path_pts = [(x0,y0), (x1,y1), …, (xn,yn)],
    find and recolor the matching Line actors in edges[].
    """
    global edges, plt
    for p0, p1 in zip(path_pts, path_pts[1:]):
        for edge in edges:
            pts = edge.points        # grab the Nx3 array
            A, B = pts[0][:2], pts[-1][:2]
            if (np.allclose(A, p0) and np.allclose(B, p1)) \
            or (np.allclose(A, p1) and np.allclose(B, p0)):
                edge.c(color)
                edge.lw(width)
                break
    plt.render()



# check if the line between x1 and x2 intersects with the obstacles
def Collision(img, p1, p2):
    """
    Return True if the line segment from p1→p2 crosses any obstacle.
    - img: 2D bool array (True=free, False=obs), origin at top-left.
    - p1, p2: (x,y) world coords, origin at bottom-left.
    """
    h, w = img.shape
    for wx, wy in bresenham(p1[0], p1[1], p2[0], p2[1]):
        row = h - 1 - wy
        col = wx
        # bounds‐check just in case
        if not (0 <= row < h and 0 <= col < w):
            return True
        if not img[row, col]:
            return True
    return False


# get the next sample point
# get the next sample point
def getNextSample(dim, samples, stepsize):
    # 1. Uniformly sample a point in [0..dim]²
    x_rand = np.array([random.randint(0, d) for d in dim], dtype=float)

    # 2. Find the nearest existing sample
    sid    = NearestSample(samples, x_rand)
    x_near = np.array(samples[sid].x, dtype=float)

    # 3. Compute the direction vector and normalize
    direction = x_rand - x_near
    norm = np.linalg.norm(direction)
    if norm == 0:
        # if the random sample coincides exactly, pick a random unit direction
        theta = random.random() * 2 * math.pi
        direction = np.array([math.cos(theta), math.sin(theta)])
    else:
        direction /= norm

    # 4. Step exactly `stepsize` along this direction
    new_pt = x_near + direction * stepsize

    # 5. Round to integer coords (so it lines up on your pixel grid)
    new_pt = np.round(new_pt).astype(int)

    # 6. Return the new Sample, linked back to its parent
    return Sample(new_pt, sid)

def choose_parent(samples, new_sample, radius, img):
    """
    Returns (best_parent_idx, best_cost) according to `method`.
    - if method=='rrt', hook to *only* the nearest neighbor
    - if method=='rrt*', search within `radius` for the lowest‐cost parent
    """
    global method
    # compute distance to every existing node
    pts   = np.array([s.x for s in samples])
    dists = np.linalg.norm(pts - new_sample.x, axis=1)

    # 1) if plain RRT, just pick the true nearest neighbor:
    if method == 'rrt':
        i = np.argmin(dists)
        return i, samples[i].cost + dists[i]

    # 2) else RRT*: look in the ball for any cheaper rewires
    #    start by seeding with the nearest anyway
    nearest_idx = np.argmin(dists)
    best_idx    = nearest_idx
    best_cost   = samples[nearest_idx].cost + dists[nearest_idx]

    # consider all within radius
    nbrs = np.where(dists <= radius)[0]
    for i in nbrs:
        if i == nearest_idx: 
            continue
        # only if collision-free from i → new_sample
        if not Collision(img, samples[i].x, new_sample.x):
            cost_via_i = samples[i].cost + dists[i]
            if cost_via_i < best_cost:
                best_cost = cost_via_i
                best_idx  = i

    return best_idx, best_cost

def rewire(samples, new_idx, radius, img):
    """For RRT*: try to rewire neighbors through samples[new_idx]."""
    new_sample = samples[new_idx]
    # gather all positions and distances to the new sample
    pts   = np.array([s.x for s in samples])
    dists = np.linalg.norm(pts - new_sample.x, axis=1)
    nbrs  = np.where((dists <= radius) & (np.arange(len(samples)) != new_idx))[0]
    for j in nbrs:
        # only if collision‐free from new_sample → samples[j]
        if not Collision(img, new_sample.x, samples[j].x):
            cost_via_new = new_sample.cost + dists[j]
            if cost_via_new < samples[j].cost:
                # remove old visual edge between j and its old parent
                old_p = samples[j].parent
                for e in edges:
                    pts_e = e.points     # Nx3 array
                    A, B = pts_e[0][:2], pts_e[-1][:2]
                    if (np.allclose(A, samples[j].x) and np.allclose(B, samples[old_p].x)) \
                    or (np.allclose(B, samples[j].x) and np.allclose(A, samples[old_p].x)):
                        plt.remove(e)
                        edges.remove(e)
                        break
                # rewire in the data
                samples[j].parent = new_idx
                samples[j].cost   = cost_via_new
                # draw the new edge
                e2 = vd.Line(samples[j].x, new_sample.x, lw=2)
                edges.append(e2)
                plt.add(e2)
    plt.render()


#%%
resent_btn = 0.0 
last_mouse_click_time = 0.0
reset_tree_click_time = 0.0
on_play_btn_time = 0.0
last_brush = None
brush_radius = 2   # will paint a (2·2+1)×(2·2+1)=5×5 block around each point
iteration_count = 0
MAX_ITERATIONS = 3000   # or whatever limit I choose
neighborRadius = 30   # or hook this up to a slider
method = 'rrt'  # or 'rrt*' if I want to implement that later

recorded_path   = []     # filled when you reach the goal: list of (pt0, pt1) pairs
path_edges = []    # global, alongside `edges` and `recorded_path`

def path_length(path):
    return sum(
        np.linalg.norm(np.array(a) - np.array(b))
        for a,b in path)


def update_stats():
    global iteration_count, samples, stats_label, plt
    stats_label.text(
        f"Iteration: {iteration_count}\n"
        f"Nodes: {len(samples)}\n")
    plt.render()



def reset_tree():
    global mode, source, samples, points, edges, reset_tree_click_time, last_mouse_click_time, iteration_count
    
    now = time.time()
    if now - last_mouse_click_time > 0.2:
        return

    if now - reset_tree_click_time < 0.1:
        return
    
    reset_tree_click_time = now
    iteration_count = 0
    mode = 'idle'
    samples[:] = [ Sample(source) ]
    samples[0].cost = 0.0
    recorded_path.clear()
    iteration_count = 0
    update_stats()


    plt.remove(points)
    for e in edges:
        plt.remove(e)
    edges.clear()

    for e in path_edges:
        plt.remove(e)
    path_edges.clear()
    recorded_path.clear()
    plt.remove('path')

    points = vd.Points([source], r=radius)
    plt.add(points)

    update_status("🔄 Tree reset")



def reset_map():
    global img, original_img, vd_img, plt
    reset_tree()  # Optional: reset tree when map resets

    img = original_img.copy()
    flipped_img = np.flipud(img).astype(np.uint8) * 255  # Flip vertically to match Vedo's convention
    new_vd_img = vd.Image(flipped_img).pickable(True)

    plt.remove(vd_img)
    plt.add(new_vd_img)
    plt.render()

    vd_img = new_vd_img  # update reference
    if validate_endpoints():
            update_status("🧼 Map reset to original")
    plt.render()



def bfunc(obj, ename):
    """
    Run/Pause button callback.
    Only fires if a real mouse-click happened recently, and is debounced.
    """
    global timer_id, mode, resent_btn, last_mouse_click_time

    now = time.time()
    # ignore all the PickEvent spam unless it's within 0.2 s of a real click
    if now - last_mouse_click_time > 0.2:
        return

    # debounce the Run/Pause action itself
    if now - resent_btn < 0.1:
        return
    resent_btn = now

    # optional: if you’re mid-draw or something, reset to idle
    if mode != 'idle':
        mode = 'idle'
        update_status("🔄 Restarting idle mode")
    else:
        update_status()

    # toggle the timer
    current = button.status().strip()
    if current == "Run":
        timer_id = plt.timer_callback("create", dt=10)
    else:
        plt.timer_callback("destroy", timer_id)

    button.switch()


def _on_play_path(btn, _):
    global last_mouse_click_time, on_play_btn_time
    now = time.time()
    if now - last_mouse_click_time > 0.2:
        return

    if now - on_play_btn_time < 0.1:
        return
    if not recorded_path:
        update_status("⚠️ No path found yet — run until you reach the goal first")
        return
    on_play_btn_time = now

    if button.status().strip()=="Pause":
        plt.timer_callback("destroy", timer_id)
        button.switch()

    for e in edges:
        plt.remove(e)
    edges.clear()

    for e in path_edges:
        plt.remove(e)
    path_edges.clear()
    plt.render()

    for a,b in recorded_path:
        seg = vd.Line(a, b, lw=4).c('green')
        plt.add(seg)
        path_edges.append(seg)
        plt.render()
        time.sleep(0.2)

    update_status("▶️ Path replay finished")


    
def doRRTIteration(event):
    global mode, stepSize, samples, points, edges, img, dest, plt, timer_id, iteration_count, MAX_ITERATIONS, method
    iteration_count += 1
    if iteration_count > MAX_ITERATIONS:
        update_status(
        f"❌ Max iterations ({MAX_ITERATIONS}) reached – no path found.\n"
        f"You can Reset the tree or map, or continue running another {MAX_ITERATIONS} iterations.")
        plt.timer_callback("destroy", timer_id)
        if button.status().strip() == "Pause":
            button.switch()
        return
    y,x = img.shape # get the image size. need to switch x and y due different conventions
    ns = getNextSample([x,y], samples, stepSize)

    # unified parent choice
    pid, cost = choose_parent(samples, ns, neighborRadius, img)

    # collision check on the *chosen* edge
    if Collision(img, samples[pid].x, ns.x):
        return

    ns.parent = pid
    ns.cost   = cost
    samples.append(ns)
    if method == 'rrt*':
        rewire(samples, len(samples)-1, neighborRadius, img)
    
    # drawing
    points.vertices = np.vstack([points.vertices, np.hstack([ns.x,0])])
    edges.append(vd.Line(ns.x, samples[ns.parent].x, lw=2))
    plt.add(points)
    plt.remove('Line')
    plt.add(edges)
    plt.render()
    if iteration_count % 10 == 0:
        update_stats()


    # stopping condition: if new node is within one step of goal and that last segment is collision‐free
    dist_to_goal = np.linalg.norm(ns.x - dest)
    if dist_to_goal <= stepSize and not Collision(img, ns.x, dest):
        final_edge = vd.Line(ns.x, dest, lw=3).c('green')
        edges.append(final_edge)
        plt.add(final_edge)

        update_status("✅ Goal reached!")   
        update_stats() 

        plt.timer_callback("destroy", timer_id)

        if button.status().strip() == "Pause":
            button.switch()

        goal_idx = len(samples) - 1
        path_pts = extract_path(samples, goal_idx, dest)
        recorded_path[:] = list(zip(path_pts[:-1], path_pts[1:]))
        for e in path_edges:
            plt.remove(e)
        path_edges.clear()

        for a,b in recorded_path:
            e = vd.Line(a,b, lw=3).c('green')
            plt.add(e)
            path_edges.append(e)
        plt.render()
        return


def update_status(msg=""):
    global mode, stepSize, mode_label, plt
    mode_label.text(f"Mode: {mode}   Step size: {stepSize}\n" + msg)
    plt.render()


def on_slider_change(widget, event):
    global stepSize
    raw = widget.value
    rounded = int(round(raw))
    # only update the slider’s internal value if it’s not already an integer
    if rounded != raw:
        widget.value = rounded
    stepSize = rounded
    update_status()


def on_keypress(evt):
    global mode, method, method_label, last_mouse_click_time
    key = evt.keypress.lower()
    if key == 'i':
        mode = 'idle'
        update_status("🛑 Back to idle")
        return

    if key == 's':
        mode = 'pick_source'
        update_status("🎯 Pick a new source")
        return

    if key == 'd':
        mode = 'pick_dest'
        update_status("🎯 Pick a new destination")
        return

    if key == 'o':
        # toggle draw mode
        if mode != 'draw_obstacle':
            mode = 'draw_obstacle'
            update_status("⬛ Draw mode – hold & drag to paint obstacles")
        else:
            mode = 'idle'
            # only validate when exiting draw mode
            if validate_endpoints():
                update_status("✅ Finished drawing obstacles")
        return

    if key == 'e':
        # toggle erase mode
        if mode != 'erase_obstacle':
            mode = 'erase_obstacle'
            update_status("⬜ Erase mode – hold & drag to remove obstacles")
        else:
            mode = 'idle'
            update_status("✅ Finished erasing obstacles")
        return
    if key == 'c':
        method = 'rrt*' if method=='rrt' else 'rrt'
        txt = "RRT*" if method == 'rrt*' else "RRT"
        method_label.text(f"Method: {txt}")
        last_mouse_click_time = time.time()
        reset_tree()  
        plt.render()
        


    # all other keys do nothing

    

def validate_endpoints():
    global source, dest, img, plt
    h, w = img.shape
    for name, pt in (('Source', source), ('Destination', dest)):
        x, y = pt
        fy = h - 1 - y   # flip back to image‐row index
        if not img[fy, x]:   # False == obstacle
            update_status(f"⚠️ {name} is now on an obstacle – press {'S' if name=='Source' else 'D'} to move")
            return False
    return True

def obstacle_brush(evt):
    global last_brush, img, vd_img, plt

    # only paint in draw or erase mode
    if mode not in ('draw_obstacle','erase_obstacle'):
        last_brush = None
        return

    pts = evt.picked3d
    if pts is None:
        last_brush = None
        return

    x, y = int(round(pts[0])), int(round(pts[1]))
    h, w = img.shape
    if not (0 <= x < w and 0 <= y < h):
        last_brush = None
        update_status("⚠️ Click out of bounds")
        return

    fy = h - 1 - y
    # decide target value: False=black for draw, True=white for erase
    target = (mode == 'erase_obstacle')

    # interpolate between last_brush and current
    if last_brush is not None:
        x0, y0 = last_brush
        fy0 = h - 1 - y0
        n = max(abs(x-x0), abs(fy-fy0)) or 1
        xs = np.linspace(x0, x, n, dtype=int)
        ys = np.linspace(fy0, fy, n, dtype=int)
    else:
        xs, ys = np.array([x]), np.array([fy])

    # paint a little square around each interpolated point
    for xi, yi in zip(xs, ys):
        for dx in range(-brush_radius, brush_radius+1):
            for dy in range(-brush_radius, brush_radius+1):
                nx, ny = xi+dx, yi+dy
                if 0 <= nx < w and 0 <= ny < h:
                    img[ny, nx] = target

    last_brush = (x, y)

    # refresh the Vedo image actor
    flipped = np.flipud(img).astype(np.uint8) * 255
    new_img = vd.Image(flipped).pickable(True)
    plt.remove(vd_img)
    plt.add(new_img)
    plt.render()
    vd_img = new_img

    update_status("⬛ Drawing obstacle…" if mode=='draw_obstacle'
                  else "⬜ Erasing obstacle…")




def add_point(evt):
    global mode, start_actor, goal_actor, source, dest, samples, points, edges, img, plt, z_offset, radius
    pts = evt.picked3d

    world_x, world_y = pts[:2]
    x, y = int(round(world_x)), int(round(world_y))
    h, w = img.shape
    if x<0 or x>=w or y<0 or y>=h:
        mode = 'idle'
        update_status("⚠️ Click out of bounds")
        return

    fy = h - 1 - y
    if not img[fy, x]:
        mode = 'idle'
        update_status("⚠️ Cannot place on obstacle!")
        return

    if mode == 'pick_source':
        plt.remove(start_actor)
        start_actor = vd.Sphere(pos=[x, y, z_offset], r=radius).c('green')
        start_actor.name = 'Start'
        plt.add(start_actor)
        source = np.array([x, y])
        reset_tree()
        update_status("Source point set.")

    elif mode == 'pick_dest':
        plt.remove(goal_actor)
        goal_actor = vd.Sphere(pos=[x, y, z_offset], r=radius).c('blue')
        goal_actor.name = 'Goal'
        plt.add(goal_actor)
        dest   = np.array([x, y]) 
        reset_tree()
        update_status("Destination point set.")


    mode = 'idle'
    update_status()


def on_click(evt):
    global mode,last_mouse_click_time
    last_mouse_click_time = time.time()
    pts = evt.picked3d

    if pts is None:
        mode = 'idle'
        update_status("⚠️ Click out of bounds")
        return
    if mode == 'draw_obstacle':
        mode = 'idle'
        if validate_endpoints():
            update_status("✅ Finished drawing obstacles")
        return

    if mode in ['pick_source', 'pick_dest']:
        return add_point(evt)
    
    mode = 'idle'
    update_status()
    plt.render()
    

    








imagePath = 'obstacle_map.png'
stepSize = 10
source = np.array([0,0])
dest = np.array([50,50])
tree = [Sample(source)]
mode = 'idle'  # 'idle' | 'pick_source' | 'pick_dest' | 'draw_obstacle' | 'erase_obstacle'

vd_img = vd.Image(imagePath).bw().binarize()
img = vd_img.tonumpy().astype(bool)
original_img = img.copy()
vd_img.pickable(True)


samples = [Sample(source)] # list to store all the node points
points = vd.Points([source]) # list to store all the node points
edges = []
plt = vd.Plotter()
plt += vd_img
plt.user_mode('2d')


z_offset = 0.1       
radius   = 5         
start_actor = vd.Sphere(pos=[source[0], source[1], z_offset], r=radius).c('green')
start_actor.name = 'Start'
goal_actor  = vd.Sphere(pos=[dest[0],   dest[1],   z_offset], r=radius).c('blue')
goal_actor.name  = 'Goal'
plt.add(start_actor)
plt.add(goal_actor)

hint_S = vd.Text2D("S: set source", pos=(0.02, 0.98), s=1, c='green')           
hint_D = vd.Text2D("D: set destination", pos=(0.15, 0.98), s=1,  c='blue')
hint_I = vd.Text2D("I: back to idle",   pos=(0.30, 0.98), s=1, c='black')
hint_O = vd.Text2D("O: draw obstacle",   pos=(0.02, 0.94), s=1, c='black')
hint_E = vd.Text2D("E: erase obstacle",   pos=(0.15, 0.94), s=1, c='black')
hint_C = vd.Text2D("C: press C to switch RRT/RRT*", pos=(0.02, 0.8), s=1, c='black')
mode_label = vd.Text2D(f"Mode: {mode}   Step size: {stepSize}",pos=(0.02, 0.9), s=1, c='white', bg='black') 
method_label = vd.Text2D(f"Method: RRT", pos=(0.02, 0.75), s=1, c='white', bg='black')
plt += hint_S
plt += hint_D
plt += hint_I
plt += hint_O
plt += hint_E
plt += hint_C
plt += mode_label
plt += method_label
update_status()
stats_label = vd.Text2D("", pos=(0.02, 0.71), c='black', bg='white', s=1)
plt.add(stats_label)

slider = plt.add_slider(on_slider_change,   1, 100, stepSize, pos=2, title="Step Size")

button = plt.add_button(bfunc, states=[" Run ","Pause"])
reset_btn = plt.add_button(lambda *args: reset_tree(), states=["Reset"], pos=(0.85, 0.1))
reset_map_btn = plt.add_button(lambda *args: reset_map(), states=["Reset Map"], pos=(0.85, 0.05))
evntid = plt.add_callback("timer", doRRTIteration, enable_picking=False)
play_btn = plt.add_button(_on_play_path, states=[" Play Path "], pos=(0.5, 0.1))
plt.add_callback("key press", on_keypress)
plt.add_callback("mouse click", on_click, enable_picking=True)
plt.add_callback("mouse move", obstacle_brush, enable_picking=True)
timer_id = -1

plt.render()
plt.show(zoom="tightest").close()

# %%