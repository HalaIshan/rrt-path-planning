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
def getNextSample(dim, samples, stepsize):
    x = np.array([random.randint(0, d) for d in dim])
    sid = NearestSample(samples,x)
    nx = np.round(x) # TODO
    ns = Sample(nx, sid)
    return ns

#%%
resent_btn = 0.0 

def reset_tree():
    global mode, source, samples, points, edges

    mode = 'idle'

    samples[:] = [ Sample(source) ]

    plt.remove(points)
    for e in edges:
        plt.remove(e)
    edges.clear()

    points = vd.Points([source], r=radius)
    plt.add(points)

    update_status("🔄 Tree reset")
    plt.render()





def bfunc(obj, ename):
    global timer_id, mode, resent_btn, samples
    if mode != 'idle':
        mode = 'idle' 
        update_status("🔄 Restarting idle mode")
    
    now = time.time()
    if now - resent_btn < 0.1:
        return
    resent_btn = now
    
    current_status = button.status().strip()
    if current_status == "Run":
        # user just clicked “Run” → start the loop
        timer_id = plt.timer_callback("create", dt=10)
    else:
        # user clicked “Pause” → stop the loop
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
    elif key == 's':
        mode = 'pick_source'
    elif key == 'd':
        mode = 'pick_dest'
    else:
        return
    update_status()



def on_click(evt):
    global mode, start_actor, goal_actor, source, dest, samples, points, edges, img, plt, z_offset, radius
    if mode not in ['pick_source', 'pick_dest']:
        return

    pts = evt.picked3d

    if pts is None:
        mode = 'idle'
        update_status("⚠️ Click out of bounds")
        return

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
    
    plt.render()








imagePath = 'obstacle_map.png'
stepSize = 10
source = np.array([0,0])
dest = np.array([50,50])
tree = [Sample(source)]
mode     = 'idle'  # 'idle' | 'pick_source' | 'pick_dest'

vd_img = vd.Image(imagePath).bw().binarize()
img = vd_img.tonumpy().astype(bool)
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
mode_label = vd.Text2D(f"Mode: {mode}   Step size: {stepSize}",pos=(0.02, 0.94), s=1, c='white', bg='black') 
plt += hint_S
plt += hint_D
plt += hint_I
plt += mode_label
update_status()

slider = plt.add_slider(on_slider_change,   1, 100, stepSize, pos=2, title="Step Size")

button = plt.add_button(bfunc, states=[" Run ","Pause"])
reset_btn = plt.add_button(lambda *args: reset_tree(), states=["Reset"], pos=(0.85, 0.1))
evntid = plt.add_callback("timer", doRRTIteration, enable_picking=False)
plt.add_callback("key press", on_keypress)
plt.add_callback("mouse click", on_click, enable_picking=True)
timer_id = -1

plt.render()
plt.show(zoom="tightest").close()

# %%