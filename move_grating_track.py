import roslib
roslib.load_manifest('flyvr')

import rospy
import geometry_msgs.msg
import std_msgs.msg
import flyvr.display_client as display_client
import flyvr

from math import sin, cos
import numpy as np

import motmot.cam_iface.cam_iface_ctypes as cam_iface
import cv2
import numpy as np
import time, sys, os
import motmot.FlyMovieFormat.FlyMovieFormat as FlyMovieFormat
import Queue
import threading
import datetime
import time
from collections import namedtuple
import cPickle as pkl
#import os

from pylab import *
from PIL import Image

#bashCommand = "cwm --rdf test.rdf --ntriples > test.nt" #
#process = subprocess.Popen(source /opt/ros/ros-flyvr.hydro/setup.bash.split(), stdout=subprocess.PIPE)
#output = process.communicate()[0]

#-----------this is the ex-matlab tracking code--------



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
    '''
    figure()
    img_marked = img.copy()
    #for ix in np.arange(circ.shape[0]):
    #    img_marked[circ[ix,0], circ[ix,1]] = 255
    for ix in np.arange(points.shape[0]):
        img_marked[points[ix,0], points[ix,1]] = 255
    imshow(img_marked, cmap='gray', interpolation='nearest')
    show()
    '''

    index = np.nonzero(points[:,0])[0]  #index=points(points(:,2)>0); but not indices

    last = np.array(  [ points[index[-1],0], points[index[-1],1] ]  ) #    last=[points(index(end),1),points(index(end),2)];
    a = last - M
    alpha = rad2deg(arccos(np.dot(a,c)/(np.linalg.norm(a)*np.linalg.norm(c))))
    if np.linalg.det(np.vstack((a,c)))<0:     #the matrix must be square!   #if right (a right of c in image, when det <0) neg angle, otherwise (left) pos angle)
        alpha = -alpha

    return alpha,points


#---------------------------- this is the stimulus and camera recording code------------------------
def save_frames(save_queue, stop_requested, filename):
    print "Saving to file: ", filename
    #code = cv2.cv.CV_FOURCC('X', 'V', 'I', 'D')
    #code = cv2.cv.CV_FOURCC('D', 'I', 'V', 'X')
    #code = cv2.cv.CV_FOURCC('P','I','M','1')
    code = cv2.cv.CV_FOURCC('M','J','P','G')
    writer = cv2.VideoWriter(filename,code,50,(1024,1024),True)
    if  writer.isOpened() == False:
        print "not open"
        sys.exit()

    counter = 0
    while True:
        counter = counter + 1
        if save_queue.empty() and stop_requested.is_set():
            print 'wrote everything, finishing up'
            break
        fnt = save_queue.get()
        frame,timestamp,caption = fnt
        cv2.putText(frame, '%6.2f s' % (timestamp/100.), (20,25), cv2.FONT_HERSHEY_SIMPLEX, 1, 255, 2)
        cv2.putText(frame, str(caption), (20,55), cv2.FONT_HERSHEY_SIMPLEX, 1, 255, 2)
        frame = cv2.cvtColor(frame,cv2.cv.CV_GRAY2RGB)
        writer.write(frame)
        #if counter > 5:
        #    counter = 0
        #    cv2.imshow('frame',frame)
        #    cv2.waitKey(1)

    writer.release()

def grab_frames(cam, save_queue, stop_requested, fit_queue):
        global i
        global caption

        cam.start_camera()
        print "Cam started"

        last_fno = None
        frametick =0
        framecount = 0
        last_fps_print = time.time()

        while not stop_requested.is_set():
            try:
                buf = np.asarray(cam.grab_next_frame_blocking())
            except cam_iface.FrameDataMissing:
                sys.stdout.write('M')
                sys.stdout.flush()
                continue
            except cam_iface.FrameSystemCallInterruption:
                sys.stdout.write('I')
                sys.stdout.flush()
                continue

            timestamp = cam.get_last_timestamp()
            fno = cam.get_last_framenumber()
            #print fno
            #if last_fno is not None:
            #    skip = (fno-last_fno)-1
            #    if skip != 0:
            #        print 'WARNING: skipped %d frames'%skip

            last_fno = fno
            if save_queue is not None:
                save_queue.put( (buf,i,caption) )
            if fit_queue is not None:
                fit_queue.put( (buf,i,caption) )

            now = time.time()
            frametick += 1
            framecount += 1

            t_diff = now-last_fps_print
            if t_diff > 20.0:
                fps = frametick/t_diff
                print "%.1f fps"%fps,
                last_fps_print = now
                frametick = 0
                #print " Videosize: " , os.path.getsize(fname)/1024.0**2, "MB"


def get_anchor_points(img):
    '''
    show img and ask user for two points
    first point: start of tail
    second point: pivot point for angle calc
    third point: tail tip
    '''
    fig = figure()
    imshow(img, cmap='gray') #imshow('video', frame)
    colorbar()
    xlabel('Image array second index')
    ylabel('Image array first index')
    points = ginput(3)
    close(fig)
    p1x, p1y = np.round(points[0])
    p2x, p2y = np.round(points[1])
    p3x, p3y = np.round(points[2])
    return np.array([p1y, p1x]), np.array([p2y, p2x]), np.array([p3y, p3x])


def do_experiment(stimulus_mode, n_iterations=1):
    rospy.init_node('grating_left_right')
    rospy.sleep(1)
    display_client.DisplayServerProxy.set_stimulus_mode('StimulusCylinderGrating')
    pub_gamma = rospy.Publisher('/ds/gamma', std_msgs.msg.Float32)
    pub_grating_info = rospy.Publisher('grating_info', flyvr.msg.CylinderGratingInfo)
    rospy.sleep(1)
    pub_gamma.publish(0)
    rospy.sleep(1)
    global i
    global caption
    i = 0
    for j in range(n_iterations):
        starttime = i
        if stimulus_mode=='left-right':
            left_right(starttime, pub_grating_info, pub_gamma)
        elif stimulus_mode=='fw-bw':
            fw_bw(starttime, pub_grating_info, pub_gamma)
        else:
            print ('Unknown stimulus mode. Exiting.')
            break
    pub_gamma.publish(0)

def get_msg():
    global phase_velocity
    ci = flyvr.msg.CylinderGratingInfo()
    ci.contrast = 1.
    ci.phase_position = 0.
    ci.reset_phase_position = False
    ci.orientation = 0.
    ci.phase_velocity = phase_velocity
    ci.wavelength = 0.35
    return ci

def left_right(starttime, pub_grating_info, pub_gamma):
    global i
    global caption
    while not rospy.is_shutdown():
            if i==starttime:
                msg = get_msg()
                pub_grating_info.publish(msg)
                pub_gamma.publish(1)
                caption = 'STIM left'
            if i==starttime+500:
               pub_gamma.publish(0)
               caption = 'STIM off'
               msg = get_msg()
               msg.phase_velocity = -1.*phase_velocity
               pub_grating_info.publish(msg)
            if i==starttime+1000:
               pub_gamma.publish(1)
               caption = 'STIM right'
            if i==starttime+1500:
               pub_gamma.publish(0)
               caption = 'STIM off'
            if i==starttime+2000:
               break
            rospy.sleep(0.01)
            i += 1


def fw_bw(starttime, pub_grating_info, pub_gamma):
    global i
    global caption
    while not rospy.is_shutdown():
            if i==starttime:
                display_client.DisplayServerProxy.set_stimulus_mode('StimulusCylinderGrating_centered_bw')
                msg = get_msg()
                pub_grating_info.publish(msg)
                pub_gamma.publish(1)
                caption = 'STIM backward'
            if i==starttime+500:
               pub_gamma.publish(0)
               caption = 'STIM off'
               display_client.DisplayServerProxy.set_stimulus_mode('StimulusCylinderGrating_centered')
               msg = get_msg()
               pub_grating_info.publish(msg)
            if i==starttime+1000:
               pub_gamma.publish(1)
               caption = 'STIM forward'
            if i==starttime+1500:
               pub_gamma.publish(0)
               caption = 'STIM off'
            if i==starttime+2000:
               break
            rospy.sleep(0.01)
            i += 1

def fit_thread_worker(p1, p2, p3, fit_queue, stop_requested, memorize_frames=False):
    global results
    while True:
        if fit_queue.empty() and stop_requested.is_set():
            print 'analyzed everything, finishing up'
            break
        fnt = fit_queue.get()
        frame,timestamp,caption = fnt
        alpha,points = fit_tail(frame, p1, p2, p3)
        if not memorize_frames:
            frame = None
        ResultTuple = namedtuple('ResultTuple', 'frame timestamp caption alpha points')
        res = ResultTuple(frame, timestamp, caption, alpha, points)
        results.append(res)

def result_list_to_arrays(results):
    #frame, timestamp, caption, alpha, points
    frames = np.array([r.frame for r in results])
    timestamps = np.array([r.timestamp for r in results])
    captions = np.array([r.caption for r in results])
    alphas = np.array([r.alpha for r in results])
    points = np.array([r.points for r in results])
    return frames, timestamps, captions, alphas, points

if __name__ == "__main__":
    global i
    i = 0
    global caption
    caption = 'preparing'
    global results
    results = []
    global phase_velocity
    phase_velocity = 3.1415

    dt = datetime.datetime.now()
    file_string = dt.strftime("%Y%m%d_%H%M%S")
    movie_fname = file_string+".avi"

    save_queue = None
    #un-comment next four lines to enable on-the-fly jpeg conpression and saving to disk
    save_queue = Queue.Queue()
    save_thread_stop_requested = threading.Event()
    save_thread = threading.Thread(target=save_frames, args=(save_queue,save_thread_stop_requested, movie_fname))
    save_thread.start()

    device_num = 0
    mode_num = 12
    num_buffers = 32
    try:
        cam = cam_iface.Camera(device_num, num_buffers, mode_num)
    except:
        print "device number out of range ... no camera found"
        sys.exit()

    cam.start_camera()
    img = np.asarray(cam.grab_next_frame_blocking())
    cam.stop_camera()
    p1,p2,p3 = get_anchor_points(img)

    fit_queue = Queue.Queue()
    fit_thread_stop_requested = threading.Event()
    fit_thread = threading.Thread(target=fit_thread_worker, args=(p1, p2, p3, fit_queue, fit_thread_stop_requested))
    fit_thread.start()

    grab_thread_stop_requested = threading.Event()
    grab_thread = threading.Thread(target=grab_frames, args=(cam, save_queue, grab_thread_stop_requested, fit_queue))
    grab_thread.start()

    #do_experiment('left-right', 1)
    do_experiment('fw-bw', 1)

    print "Stopping grab thread"
    grab_thread_stop_requested.set()
    grab_thread.join()

    print "Stopping fit thread"
    fit_thread_stop_requested.set()
    fit_thread.join()

    try:
        print "Stopping save thread"
        save_thread_stop_requested.set()
        save_thread.join()
    except:
        pass

    frames, timestamps, captions, alphas, points = result_list_to_arrays(results)
    import matplotlib.pyplot as plt
    plt.figure()
    plot(timestamps, alphas)
    plt.show()
    savefig(file_string + '.pdf')
    fo = open(file_string + '.pkl', 'wb')
    fo2 = open(file_string + '_data.pkl', 'wb')
    pkl.dump({'frames':frames, 'timestamps':timestamps, 'captions':captions, 'alphas':alphas, 'points':points}, fo)
    #pkl.dump(flyvr.msg.CylinderGratingInfo(contrast), fo2)
    #pkl.dump([flyvr.msg.CylinderGratingInfo(contrast), ci.phase_position, ci.reset_phase_position, ci.phase_velocity, ci.orientation, ci.wavelength], fo2)
    ############HOW TO DUMP THE PARAMETERS? pkl.dump(ci.contrast, ci.reset_phase_position, ci.orientation, ci.phase_velocity, ci.wavelength, fo2)
    fo.close()
