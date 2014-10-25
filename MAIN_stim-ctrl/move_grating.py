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
import numpy as nx
import time, sys, os
import motmot.FlyMovieFormat.FlyMovieFormat as FlyMovieFormat
import Queue
import threading
from datetime import datetime
import time
#import os

#26
#bashCommand = "cwm --rdf test.rdf --ntriples > test.nt" #
#process = subprocess.Popen(source /opt/ros/ros-flyvr.hydro/setup.bash.split(), stdout=subprocess.PIPE)
#output = process.communicate()[0]

def save_frames(save_queue, stop_requested):
    dt = datetime.now()
    fileString = dt.strftime("%Y%m%d_%H%M%S")
    fname = fileString+".avi"
    print "Saving to file: ", fname
    #code = cv2.cv.CV_FOURCC('X', 'V', 'I', 'D')
    #code = cv2.cv.CV_FOURCC('D', 'I', 'V', 'X')
    #code = cv2.cv.CV_FOURCC('P','I','M','1')
    code = cv2.cv.CV_FOURCC('M','J','P','G')
    writer = cv2.VideoWriter(fname,code,50,(1024,1024),True)
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

def grab_frames(device_num, mode_num, num_buffers, save_queue, stop_requested):

        global i
        global caption
        if mode_num is None:
            mode_num=0
        try:
            cam = cam_iface.Camera(device_num,num_buffers,mode_num)
        except:
            print "device number out of range ... no camera found"
            sys.exit()

        cam.start_camera()
        print "Cam started"

        last_fno = None
        frametick =0
        framecount = 0
        last_fps_print = time.time()

        while not stop_requested.is_set():
            try:
                buf = nx.asarray(cam.grab_next_frame_blocking())
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

            last_fno=fno
            if True:
                save_queue.put( (buf,i,caption) )

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

def do_experiment(stimulus_mode, n_iterations=2):
    #os.system('source /opt/ros/ros-flyvr.hydro/setup.bash; roslaunch /opt/ros/ros-flyvr.hydro/flyvr/launch/fishvr_display_server_grating.launch')

    '''
    package = 'flyvr'
    executable = 'flyvr'
    node = roslaunch.core.Node(package, executable)

    launch = roslaunch.scriptapi.ROSLaunch()
    launch.start()

    process = launch.launch(display_server)
    print process.is_alive()
    '''

    rospy.init_node('grating_left_right')
    rospy.sleep(1)
    display_client.DisplayServerProxy.set_stimulus_mode('StimulusCylinderGrating')
    pub_gamma = rospy.Publisher('/ds/gamma', std_msgs.msg.Float32)
    pub_grating_info = rospy.Publisher('grating_info', flyvr.msg.CylinderGratingInfo)
    rospy.sleep(1)
    phase_velocity = 3.1415


    def left_right():
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
    def fw_bw():
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


    def get_msg():
        ci = flyvr.msg.CylinderGratingInfo()
        ci.contrast = 1.
        ci.phase_position = 0.
        ci.reset_phase_position = False
        ci.orientation = 0.
        ci.phase_velocity = phase_velocity
        ci.wavelength = 0.35
        return ci

    pub_gamma.publish(0)
    rospy.sleep(1)
    global i
    global caption
    i = 0
    for j in range(n_iterations):
        starttime = i
        if stimulus_mode=='left-right':
            left_right()

        elif stimulus_mode=='fw-bw':
            fw_bw()

        else:
            print 'Unknown stimulus mode. Exiting.'
            break


    pub_gamma.publish(0)

if __name__ == "__main__":
    global i
    i = 0
    global caption
    caption = 'preparing'

    save_queue = Queue.Queue()
    save_thread_stop_requested = threading.Event()
    save_thread = threading.Thread(target=save_frames, args=(save_queue,save_thread_stop_requested))
    save_thread.start()

    grab_thread_stop_requested = threading.Event()
    grab_thread = threading.Thread(target=grab_frames, args=(0,12,32,save_queue,grab_thread_stop_requested))
    grab_thread.start()

    #do_experiment('left-right', 2)
    do_experiment('fw-bw', 2)

    grab_thread_stop_requested.set()
    grab_thread.join()
    save_thread_stop_requested.set()
    save_thread.join()
