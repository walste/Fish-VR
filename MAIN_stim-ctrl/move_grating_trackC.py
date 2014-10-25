import roslib
roslib.load_manifest('flyvr')
import rospy
import std_msgs.msg
import flyvr.display_client as display_client
import flyvr
import numpy as np
import motmot.cam_iface.cam_iface_ctypes as cam_iface
import Queue
import threading
import datetime
import time
from collections import namedtuple
from pylab import *
from fishvr_utils import *

#bashCommand = "cwm --rdf test.rdf --ntriples > test.nt" #
#process = subprocess.Popen(source /opt/ros/ros-flyvr.hydro/setup.bash.split(), stdout=subprocess.PIPE)
#output = process.communicate()[0]


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


def do_experiment(params):
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
    for j in range(params.n_iterations):
        starttime = i
        if params.stimulus_mode=='left-right':
            left_right(starttime, pub_grating_info, pub_gamma, params)
        elif params.stimulus_mode=='bw-fw':
            bw_fw(starttime, pub_grating_info, pub_gamma, params)
        elif params.stimulus_mode == 'fw_acc':
            fw_acc(starttime, pub_grating_info, pub_gamma, params)
        else:
            print ('Unknown stimulus mode. Exiting.')
            break
    pub_gamma.publish(0)


def left_right(starttime, pub_grating_info, pub_gamma, params):
    global i
    global caption
    while not rospy.is_shutdown():
            if i==starttime:
                msg = get_msg(params)
                pub_grating_info.publish(msg)
                pub_gamma.publish(1)
                caption = 'STIM left'
            if i==starttime+params.change_offsets[0]:
               pub_gamma.publish(0)
               caption = 'STIM off'
               msg = get_msg(params)
               msg.phase_velocity = -1.*params.phase_velocity
               pub_grating_info.publish(msg)
            if i==starttime+params.change_offsets[1]:
               pub_gamma.publish(1)
               caption = 'STIM right'
            if i==starttime+params.change_offsets[2]:
               pub_gamma.publish(0)
               caption = 'STIM off'
            if i==starttime+params.change_offsets[3]:
               break
            rospy.sleep(0.01)
            i += 1


def bw_fw(starttime, pub_grating_info, pub_gamma, params):
    global i
    global caption
    while not rospy.is_shutdown():
            if i==starttime:
                display_client.DisplayServerProxy.set_stimulus_mode('StimulusCylinderGrating_centered_bw')
                msg = get_msg(params)
                msg.phase_velocity = params.phase_velocity_acc
                pub_grating_info.publish(msg)
                pub_gamma.publish(1)
                caption = 'STIM backward'
            if i==starttime+params.change_offsets[0]:
               pub_gamma.publish(0)
               caption = 'STIM off'
               display_client.DisplayServerProxy.set_stimulus_mode('StimulusCylinderGrating_centered')
               msg = get_msg(params)
               msg.phase_velocity = params.phase_velocity_acc
               pub_grating_info.publish(msg)
            if i==starttime+params.change_offsets[1]:
               pub_gamma.publish(1)
               caption = 'STIM forward'
            if i==starttime+params.change_offsets[2]:
               pub_gamma.publish(0)
               caption = 'STIM off'
            if i==starttime+params.change_offsets[3]:
               break
            rospy.sleep(0.01)
            i += 1

def fw_acc(starttime, pub_grating_info, pub_gamma, params):
    global i
    global caption
    msg = get_msg(params)
    while not rospy.is_shutdown():
        if i==starttime:
            display_client.DisplayServerProxy.set_stimulus_mode('StimulusCylinderGrating_centered')
            msg.phase_velocity = params.phase_velocity_acc
            pub_gamma.publish(1)
        if i < (starttime + params.change_offsets[1]):
            msg.phase_velocity += params.phase_velocity_increment_per_10ms
            pub_grating_info.publish(msg)
            caption = 'STIM forward, accelerating'
        if i==starttime+params.change_offsets[1]:
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
        alpha,points,circ,pos,maxval = fit_tail(frame, p1, p2, p3)
        if not memorize_frames:
            frame = None
        ResultTuple = namedtuple('ResultTuple', 'frame timestamp caption alpha points circ pos maxval')
        res = ResultTuple(frame, timestamp, caption, alpha, points, circ, pos,maxval)
        if not alpha is np.nan:
            res = ResultTuple(None, timestamp, caption, alpha, points, circ, pos,maxval)
        results.append(res)


if __name__ == "__main__":
    global i
    i = 0
    global caption
    caption = 'preparing'
    global results
    results = []

    params = Params()
    params.phase_velocity_acc = 5.0
    params.phase_velocity_increment_per_10ms = 0.01
    params.stimulus_mode = 'bw-fw'
    params.n_iterations = 1
    params.set_brightness = 10
    params.set_gamma =1200
    params.set_exposure = 150
    #params.change_offsets = [800, 1300, 1800, 2300]

    dt = datetime.datetime.now()
    file_string = dt.strftime("%Y%m%d_%H%M%S")
    file_string = "../fishVR_data/" + file_string
    movie_fname = file_string + params.stimulus_mode + ".avi"
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

    cam.set_camera_property(1, params.set_exposure, 0) # Exposure
    cam.set_camera_property( 0, params.set_brightness, 0) # Brightness
    cam.set_camera_property(6, params.set_gamma, 0) # Gamma
    cam.start_camera()
    img = np.asarray(cam.grab_next_frame_blocking())
    cam.stop_camera()
    p1,p2,p3 = get_anchor_points(img)

    fit_queue = Queue.Queue()
    fit_thread_stop_requested = threading.Event()
    fit_thread = threading.Thread(target=fit_thread_worker, args=(p1, p2, p3, fit_queue, fit_thread_stop_requested, True))
    fit_thread.start()

    grab_thread_stop_requested = threading.Event()
    grab_thread = threading.Thread(target=grab_frames, args=(cam, save_queue, grab_thread_stop_requested, fit_queue))
    grab_thread.start()

    do_experiment(params)

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
    #cam.stop_camera()
    cam.close()
    del cam

    print "Assembling results"
    res = Results()
    res.params = params
    res.from_results_list(results)
    del results
    print "Plotting"
    res.plot_alphas(file_string)
    print "Saving frames"
    print res.frames.shape
    np.save(file_string + '.npy', res.frames)
    res.frames = None
    print "Saving results"
    params.save(file_string)
    res.save(file_string)
    print "Done, returning."
