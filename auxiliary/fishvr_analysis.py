import numpy as np
import pylab as pl
import fishvr_utils
import datetime
import matplotlib

filename_no_ext = '20140917_204337'

frames = np.load(filename_no_ext + '.npy')
res = fishvr_utils.Results()
res.load(filename_no_ext + '.pkl')
nans = np.isnan(res.alphas)
nanix = np.argwhere(nans)

cmap = matplotlib.cm.gray
cmap.set_bad('r',1.)

for i,ix in enumerate(nanix):
    points = np.squeeze(res.points[ix])
    maxval = res.maxvals[ix]
    pos = res.poss[ix]
    circ = res.circs[ix]
    print points, maxval, pos, circ
    img = frames[i]

    pl.figure()
    img_marked = img.astype(np.float32)
    #for inx in np.arange(circ.shape[0]):
    #    img_marked[circ[inx,0], circ[inx,1]] = 255
    for inx in np.arange(points.shape[0]):
        img_marked[points[inx,0], points[inx,1]] = np.nan
    pl.imshow(img_marked, cmap=cmap, interpolation='nearest')
    pl.draw()
    dt = datetime.datetime.now()
    file_string = dt.strftime("%Y%m%d_%H%M%S")
    #savefig('fitting_end_plot_' + file_string + '.png')
pl.show(block=False)

np.save('error_frame_example.npy', frame)
