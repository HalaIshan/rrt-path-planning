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
    def __init__(self, x, parent = None):
        self.x = x
        self.parent = parent

# find the nearest sample using KDTree
def NearestSample(samples, x):
    tree = KDTree([sample.x for sample in samples])
    dist, sid = tree.query(x)
    return sid


# check if the line between x1 and x2 intersects with the obstacles
def Collision(img, x1, x2): 
    pass # TODO

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


#%%
resent_btn = 0.0 
last_mouse_click_time = 0.0
reset_tree_click_time = 0.0
last_brush = None
brush_radius = 2   # will paint a (2·2+1)×(2·2+1)=5×5 block around each point



def reset_tree():
    global mode, source, samples, points, edges, reset_tree_click_time, last_mouse_click_time

    now = time.time()
    if now - last_mouse_click_time > 0.2:
        return

    if now - reset_tree_click_time < 0.1:
        return
    
    reset_tree_click_time = now

    mode = 'idle'
    samples[:] = [ Sample(source) ]

    plt.remove(points)
    for e in edges:
        plt.remove(e)
    edges.clear()

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

    
    

def doRRTIteration(event):
    y,x = img.shape # get the image size. need to switch x and y due different conventions
    ns = getNextSample([x,y], samples, stepSize)
    
    # test for collision going from p to nn
    if Collision(img, ns.x, dest): # there is a collision
        return
    # no collision, we can add the new node
    samples.append(ns)
    
    # drawing
    points.vertices = np.vstack([points.vertices, np.hstack([ns.x,0])])
    edges.append(vd.Line(ns.x, samples[ns.parent].x, lw=2))
    plt.add(points)
    plt.remove('Line')
    plt.add(edges)
    plt.render()

    # check if we have reached the destination
    if False: # TODO
        print('stop')


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
    global mode
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
mode_label = vd.Text2D(f"Mode: {mode}   Step size: {stepSize}",pos=(0.02, 0.9), s=1, c='white', bg='black') 
plt += hint_S
plt += hint_D
plt += hint_I
plt += hint_O
plt += hint_E
plt += mode_label
update_status()

slider = plt.add_slider(on_slider_change,   1, 100, stepSize, pos=2, title="Step Size")

button = plt.add_button(bfunc, states=[" Run ","Pause"])
reset_btn = plt.add_button(lambda *args: reset_tree(), states=["Reset"], pos=(0.85, 0.1))
reset_map_btn = plt.add_button(lambda *args: reset_map(), states=["Reset Map"], pos=(0.85, 0.05))
evntid = plt.add_callback("timer", doRRTIteration, enable_picking=False)
plt.add_callback("key press", on_keypress)
plt.add_callback("mouse click", on_click, enable_picking=True)
plt.add_callback("mouse move", obstacle_brush, enable_picking=True)
timer_id = -1

plt.render()
plt.show(zoom="tightest").close()

# %%