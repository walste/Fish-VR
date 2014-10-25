from pylab import *
from PIL import Image
import numpy as np
#from collections import namedtuple


def get_anchor_points(img):
    '''
    show img and ask user for two points
    first point: start of tail
    second point: pivot point for angle calc
    third point: tail tip
    '''
    fig = figure()
    imshow(img, cmap='gray')
    colorbar()
    xlabel('Image array second index')
    ylabel('Image array first index')
    points = ginput(3)
    close(fig)
    p1x, p1y = np.round(points[0])
    p2x, p2y = np.round(points[1])
    p3x, p3y = np.round(points[2])
    return np.array([p1y, p1x]), np.array([p2y, p2x]), np.array([p3y, p3x])


def fit_tail(img, p1, p2, p3,
            veclength=200, tail_fit_step_px=20, tail_direction=[-1,0],
            hor_res=2*1024, vert_res=2*1024,
            intens_thresh=100):
    taillength = np.linalg.norm(p3-p2)
    n = np.ceil(taillength / tail_fit_step_px)
    r = tail_fit_step_px
    t = np.arange(-veclength/2., veclength/2.)
    M = p2
    c = p3-p2
    rr = tail_direction

    points = np.zeros((n,2))
    pos = np.zeros((n-1))
    maxval = pos.copy()

    points[0,:] = p1   #x,y wie im Bild!!
    k = 1
    circ = np.zeros((veclength,3)) #kreislaenge=veclength+1=length(a)
    circ[:,0] = points[k-1,0] + r*rr[0]
    circ[:,0] += t*rr[1]
    circ[:,0] = np.round(circ[:,0])
    circ[:,1] = points[k-1,1] + r*rr[1]
    circ[:,1] += t*rr[0]
    circ[:,1] = np.round(circ[:,1])

    for i in range(veclength):
        if (circ[i][1]<=0) or (vert_res<=circ[i][1]) or (circ[i,0]<=0) or (hor_res<=circ[i,0]):
            break
        circ[i,2] = img[circ[i,0],circ[i,1]]

    maxval[k-1] = np.max(circ[:,2])
    pos[k-1] = np.argmax(circ[:,2])

    points[k,:] = [circ[pos[k-1],0], circ[pos[k-1],1]]
    for k in np.arange(2, n):
        #update walking direction
        A = points[k-1,:] - points[k-2,:]
        rr = A/np.linalg.norm(A)
        #print rr

        #generate list of pixels in a slab normal to walking direction
        circ[:,0] = points[k-1,0] + r*rr[0]
        circ[:,0] += t*rr[1]
        circ[:,0] = np.round(circ[:,0])

        circ[:,1] = points[k-1,1] + r*rr[1]
        circ[:,1] += t*rr[0]
        circ[:,1] = np.round(circ[:,1])

        for i in np.arange(1, veclength):
            if (circ[i][1]<0) or (vert_res<=circ[i][1]) or (circ[i,0]<0) or (hor_res<=circ[i,0]):
                break
            circ[i,2] = img[circ[i,0],circ[i,1]]

        maxval[k-1] = np.max(circ[:,2])
        pos[k-1] = np.argmax(circ[:,2])

        if (maxval[k-2]<intens_thresh or (circ[i,1]<0) or (vert_res<=circ[i,1]) or (circ[i,0]<0) or (hor_res<=circ[i,0])):
            break
        points[k,:] = [ circ[pos[k-1],0], circ[pos[k-1],1] ]

    figure()
    img_marked = img.copy()
    #for ix in np.arange(circ.shape[0]):
    #    img_marked[circ[ix,0], circ[ix,1]] = 255
    for ix in np.arange(points.shape[0]):
        img_marked[points[ix,0], points[ix,1]] = 255
    imshow(img_marked, cmap='gray', interpolation='nearest')
    show()

    index = np.nonzero(points[:,0])[0]  #index=points(points(:,2)>0); but not indices

    last = np.array(  [ points[index[-1],0], points[index[-1],1] ]  ) #    last=[points(index(end),1),points(index(end),2)];
    a = last - M
    alpha = rad2deg(arccos(np.dot(a,c)/(np.linalg.norm(a)*np.linalg.norm(c))))
    if np.linalg.det(np.vstack((a,c)))<0:     #the matrix must be square!   #if right (a right of c in image, when det <0) neg angle, otherwise (left) pos angle)
        alpha = -alpha

    return alpha,points


if __name__ == '__main__':
    infile_name = 'test/light_left_00000.bmp'
    pil_img = Image.open(infile_name)
    img = np.array(pil_img)

    #testing coordinate system orientation
    #img = zeros((2,3))
    #img[1,0] = 1

    p1,p2,p3 = get_anchor_points(img)
    #p1,p2,p3 = (1007.0, 1478.0), (970.0, 1358.0), (949.0, 919.0)
    print((p1,p2,p3))


    alpha = fit_tail(img, p1, p2, p3)
    print(alpha)
