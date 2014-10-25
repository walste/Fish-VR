import numpy as np
import pylab as pl
import fishvr_utils
import datetime

img = np.load('error_frame_example.npy')
p1,p2,p3 = fishvr_utils.get_anchor_points(img)
alpha,points,circ,pos,maxval = fishvr_utils.fit_tail(img, p1, p2, p3)

cmap = matplotlib.cm.gray
cmap.set_bad('r',1.)

pl.figure()
img_marked = img.astype(np.float32)
for ix in np.arange(circ.shape[0]):
    img_marked[circ[ix,0], circ[ix,1]] = np.nan
for ix in np.arange(points.shape[0]):
    img_marked[points[ix,0], points[ix,1]] = np.nan
pl.imshow(img_marked, cmap=cmap, interpolation='nearest')
pl.show(block=False)
dt = datetime.datetime.now()
file_string = dt.strftime("%Y%m%d_%H%M%S")
#savefig('fitting_error_plot_' + file_string + '.png')




