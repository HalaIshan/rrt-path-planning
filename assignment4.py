#%%
from scipy.spatial import KDTree
import numpy as np
import math
import random
import vedo as vd
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
def bfunc(obj, ename):
    global timer_id
    plt.timer_callback("destroy", timer_id)
    if "Run" in button.status():
        timer_id = plt.timer_callback("create", dt=10)
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

imagePath = 'obstacle_map.png'
stepSize = 10
source = np.array([0,0])
dest = np.array([50,50])
tree = [Sample(source)]

vd_img = vd.Image(imagePath).bw().binarize()
img = vd_img.tonumpy().astype(bool)

samples = [Sample(source)] # list to store all the node points
points = vd.Points([source]) # list to store all the node points
edges = []
plt = vd.Plotter()
plt += vd_img
plt.user_mode('2d')
button = plt.add_button(bfunc, states=[" Run ","Pause"])
evntid = plt.add_callback("timer", doRRTIteration, enable_picking=False)
timer_id = -1

plt.show(zoom="tightest").close()

# %%