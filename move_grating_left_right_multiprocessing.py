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
import multiprocessing
from multiprocessing import Process, Queue, Value
from datetime import datetime
import time

'''
Tobias' note to self:
- multiprocessing is not necessarily faster than multithreading with GIL: interprocess comm is much slower than shared mem (frames have to be serialized...)
- try the numpy shared mem module:
-- create a pool of shared mem frames
-- use interprocess comm to manage them: writer tells which ones are full, reader tells which ones are empty. if out of space: allocate more
- Q: how to communicate timestamps and captions? array of shmem slots?
'''

def save_frames(save_queue, stop_requested):
    dt = datetime.now()
    fileString = dt.strftime("%Y%m%d_%H%M%S")
    fname = fileString+".avi"
    print "Saving to file: ", fname
    #code = cv2.cv.CV_FOURCC('X', 'V', 'I', 'D')
    #code = cv2.cv.CV_FOURCC('D', 'I', 'V', 'X')
    #code = cv2.cv.CV_FOURCC('P','I','M','1')
    code = cv2.cv.CV_FOURCC('M','J','P','G')
    writer = cv2.VideoWriter(fname,code,25,(1024,1024),True)
    if  writer.isOpened() == False:
        print "not open"
        sys.exit()
    
    counter = 0
    while True:
        counter = counter + 1
        if save_queue.empty() and stop_requested.is_set():
            print 'wrote everything, finishing up'
            break
        try:
            fnt = save_queue.get_nowait()
            frame,timestamp,caption = fnt
            cv2.putText(frame, '%6.2f s' % (timestamp/100.), (20,50), cv2.FONT_HERSHEY_SIMPLEX, 1, 255, 2)
            cv2.putText(frame, str(caption), (20,55), cv2.FONT_HERSHEY_SIMPLEX, 1, 255, 2)
            frame = cv2.cvtColor(frame,cv2.cv.CV_GRAY2RGB)  
            writer.write(frame)
            #if counter > 5:
            #    counter = 0
            #    cv2.imshow('frame',frame)
            #    cv2.waitKey(1)
        except:
            pass
    
    writer.release()
    print 'Saver process exiting'
        
def grab_frames(device_num, mode_num, num_buffers, save_queue, caption_queue, timestamp_shmem, stop_requested):
    print 'grab start'
    caption = ''
    if mode_num is None:
        mode_num=0
    try:
        print 'grab cam'
        cam = cam_iface.Camera(device_num,num_buffers,mode_num)
    except:
        print "device number out of range ... no camera found"
        sys.exit()

    print 'grab cam start'
    cam.start_camera()
    print "Cam started"

    last_fno = None
    frametick =0
    framecount = 0
    last_fps_print = time.time()

    print 'grab loop start'
    while not stop_requested.is_set():
        try:
            buf = nx.asarray(cam.grab_next_frame_blocking())
            #print 'f'
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
        try:
            caption = caption_queue.get_nowait()
        except Exception,e:
            pass
        save_queue.put( (buf, timestamp_shmem.value, caption) )
        
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
    print 'grabber exiting'

def do_experiment(caption_queue, timestamp_shmem):
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
    i = 0L
    for j in range(1):
        starttime = i
        while not rospy.is_shutdown():
            timestamp_shmem.value = i
            if i==starttime:
                msg = get_msg()
                pub_grating_info.publish(msg)
                pub_gamma.publish(1)
                caption_queue.put_nowait('STIM left')
            if i==starttime+500:
               pub_gamma.publish(0)
               caption_queue.put_nowait('STIM off')
               msg = get_msg()
               msg.phase_velocity = -1.*phase_velocity
               pub_grating_info.publish(msg)
            if i==starttime+1000:
               pub_gamma.publish(1)
               caption_queue.put_nowait('STIM right')
            if i==starttime+1500:
               pub_gamma.publish(0)
               caption_queue.put_nowait('STIM off')
            if i==starttime+2000:
               break
            rospy.sleep(0.01)
            i += 1
    pub_gamma.publish(0)  

if __name__ == "__main__":
    timestamp_shmem = Value('L', 0L, lock=False)
    caption_queue = Queue()
    caption_queue.put_nowait('preparing')
    
    save_queue = Queue()
    save_thread_stop_requested = multiprocessing.Event()
    save_thread = multiprocessing.Process(target=save_frames, args=(save_queue,save_thread_stop_requested))
    save_thread.start()
    
    grab_thread_stop_requested = multiprocessing.Event()
    grab_thread = multiprocessing.Process(target=grab_frames, args=(0,12,32, save_queue, caption_queue, timestamp_shmem, grab_thread_stop_requested))
    grab_thread.start()
    
    do_experiment(caption_queue, timestamp_shmem)
    
    print 'Asking grabbing process to stop'
    grab_thread_stop_requested.set()
    grab_thread.join()
    print 'Grabbing process stopped'
    print 'Asking saving process to stop'
    save_thread_stop_requested.set()
    save_thread.join()
    print 'Saving process stopped'
