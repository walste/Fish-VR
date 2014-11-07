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
from fishvr_utils_imsaving2 import *
from pyfirmata import ArduinoMega, util
import cv2
import streamingpickle as spkl

#bashCommand = "cwm --rdf test.rdf --ntriples > test.nt" #
#process = subprocess.Popen(source /opt/ros/ros-flyvr.hydro/setup.bash.split(), stdout=subprocess.PIPE)
#output = process.communicate()[0]


def grab_frames(cam, save_queue, stop_requested, fit_queue):
        global i
        global caption
        global roi #(x0,x1,y0,y1)
        global vid_observer

        cam.start_camera()
        print "Cam started"

        last_fno = None
        frametick =0
        framecount = 0
        last_fps_print = time.time()

        while not stop_requested.is_set():
            try:
                buf = np.asarray(cam.grab_next_frame_blocking())
                buf_cropped = buf[roi[0]:roi[1], roi[2]:roi[3]]

            except cam_iface.FrameDataMissing:
                sys.stdout.write('M')
                sys.stdout.flush()
                continue
            except cam_iface.FrameSystemCallInterruption:
                sys.stdout.write('I')
                sys.stdout.flush()
                continue
            except Exception as e:
                print e

            timestamp = cam.get_last_timestamp()
            fno = cam.get_last_framenumber()
            #print fno
            #if last_fno is not None:
            #    skip = (fno-last_fno)-1
            #    if skip != 0:
            #        print 'WARNING: skipped %d frames'%skip

            last_fno = fno

            if save_queue is not None:
                save_queue.put( (buf_cropped,i,caption,vid_observer) )
            if fit_queue is not None:
                fit_queue.put( (buf_cropped,i,caption) )

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
    rospy.init_node('classical_conditioning')
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
        elif params.stimulus_mode == 'idle_grating':
            idle_grating(starttime, pub_grating_info, pub_gamma, params, board)
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

def idle_grating(starttime, pub_grating_info, pub_gamma, params, board):
    global i
    global caption
    global vid_observer
    msg = get_msg(params)
    msg.phase_velocity = 0
    #msg.phase_velocity = 5
    pub_grating_info.publish(msg)
    r = rospy.Rate(100)
    while not rospy.is_shutdown():
        delta = i - starttime
        if delta%params.ITI >= (params.ITI - params.vid_span):
            vid_observer = 1
            caption = 'Observing before'
        if delta%params.ITI < params.dur_CS:
            pub_gamma.publish(1)
            caption = 'CS on, idle grating'
            vid_observer = 1        # The movie recording should start when vid_observer is set to 1 (10s before and after CS and during CS)
                                       # and end when it's set to 0 again (10s after CS ends).
        if (delta%params.ITI == params.onset_laser) and (delta//params.ITI in params.trials):
            board.digital[53].write(1) #switch laser on
        if delta%params.ITI == params.dur_CS+1:
            board.digital[53].write(0) #switch laser off
            pub_gamma.publish(0)
        if delta%params.ITI > params.dur_CS and delta%params.ITI < params.dur_CS+params.vid_span:
            caption = "Observing after"
        if delta%params.ITI == params.vid_span + params.dur_CS:        #Here the observing movie should stop again.
            vid_observer = 0
        if delta == (params.ITI*params.exp_iterations)+params.dur_CS:    #added + params.dur_CS because only twelve iterations were performed (instead of 13)
            break
        r.sleep()
        i += 1
        #if i%1000==0:
            #print 'i: %8d   time: %s' % (i,str(time.time()))


def fit_thread_worker(p1, p2, p3, fit_queue, stop_requested,  results_fname, debugmode=False, memorize_frames=False):

    file_results = open(results_fname, 'wb')
    while True:
        if fit_queue.empty() and stop_requested.is_set():
            print 'analyzed everything, finishing up'
            file_results.close()
            break
        try:
            fnt = fit_queue.get(block=True, timeout=1)
        except Queue.Empty as e:
            continue
        frame, timestamp, caption = fnt
        alpha, points, circ, pos, maxval = fit_tail(frame, p1, p2, p3)
        if not debugmode:
            result_tuple = (timestamp, alpha)
        else:
            result_tuple = (timestamp, alpha, points, circ, pos, maxval)
        spkl.s_dump_elt(result_tuple, file_results) # the 0 item of results_fname tuple is timestamp=int, the 1st item is alpha=np.float 64

        if not memorize_frames:
            frame = None


if __name__ == "__main__":
    global i
    i = 0
    global caption
    caption = 'preparing'
    global roi
    #roi = (50,150,400,450()
    roi = (200, 600, 400, 800)
    global vid_observer
    vid_observer = 1
    device_num = 0
    mode_num = 12
    num_buffers = 32

    stimulus_statement = '''Please enter stimulus mode:
        1 = idle grating, 2 = grating left-right,
        3 = grating backward-forward, 4 = grating forward accelerating
        '''
    params = Params()
    params.phase_velocity_acc = 5.0
    params.phase_velocity_increment_per_10ms = 0.01
    params.n_iterations = 1
    params.set_brightness = 10
    params.set_gamma =1200
    params.set_exposure = 150
    params.change_offsets = [500, 1000, 1500, 2000]

    #params.ITI = 6.*60*100
    params.ITI = 0.5*60*100 #*6 instead of 0.25 # The inter trial interval (ITI) = 6 min, time between individual trial onsets
    #params.exp_iterations = 13  #Number of iterations consisting of pre-phase (1), trials (10) and tests (3)
    params.exp_iterations = 1
    #params.onset_laser = 3. *100 # The laser starts 3s into the CS and lasts 2s.
    params.onset_laser = 1. *100
    params.trials = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10] # the laser is on the trials (not in pre and not in test)
    #params.dur_CS = 5. *100  #The CS lasts 5s.
    params.dur_CS = 2. *100
    #params.vid_span = 10. * 100 #The observation should span 10s before and after the CS, which is being recorded.
    params.vid_span = 2. * 100
    try:
        cam = cam_iface.Camera(device_num, num_buffers, mode_num )
    except:
        print "device number out of range ... no camera found"
        sys.exit()

    cam.set_camera_property(1, params.set_exposure, 180) # Exposure
    cam.set_camera_property( 0, params.set_brightness, 0) # Brightness
    cam.set_camera_property(6, params.set_gamma, 1200) # Gamma
    cam.start_camera()
    img = np.asarray(cam.grab_next_frame_blocking())
    img_cropped = img[roi[0]:roi[1], roi[2]:roi[3]]
    cam.stop_camera()
    start_statement = ''' Please enter which mode you wish to access.
                        1 = view only, 2 = experiment mode
                    '''
    is_valid_mode = 0
    while not is_valid_mode:
        try:
            starting_mode = int(raw_input(start_statement))
            is_valid_mode = 1
        except ValueError, e:
             print ("'%s' is not a valid input." % e.args[0].split(": ")[1])


    if starting_mode == 1:
        ''' start the grab thread and the adjusted safe thread that ONLY VIEWS here.'''
        print 'You chose view only mode.'
        save_queue = Queue.Queue()
        fit_queue = None
        movie_fname = None
        save_thread_stop_requested = threading.Event()
        save_thread = threading.Thread(target=save_frames, args=(save_queue,save_thread_stop_requested, movie_fname, roi, starting_mode))
        save_thread.start()

        grab_thread_stop_requested = threading.Event()
        grab_thread = threading.Thread(target=grab_frames, args=(cam, save_queue, grab_thread_stop_requested, fit_queue))
        grab_thread.start()

        save_thread.join()

        print "Stopping grab thread"
        grab_thread_stop_requested.set()
        grab_thread.join()

        cam.close()
        del cam
        sys.exit()

    if starting_mode == 2:
        print 'You chose experiment mode.'
        is_valid_stim = 0
        while not is_valid_stim:
            try:
                k = int(raw_input(stimulus_statement))
                is_valid_stim = 1
            except ValueError, e:
                 print ("'%s' is not a valid input." % e.args[0].split(": ")[1])
        if k == 1:
            params.stimulus_mode = 'idle_grating'
        elif k == 2:
            params.stimulus_mode = 'left-right'
        elif k == 3:
            params.stimulus_mode = 'bw-fw'
        elif k == 4:
            params.stimulus_mode = 'fw_acc'
        else:
            print ("Invalid stimulus input. Please try again.")

        dt = datetime.datetime.now()
        file_string = dt.strftime("%Y%m%d_%H%M%S")
        file_string = "../fishVR_data/" + file_string
        movie_fname = file_string + params.stimulus_mode + ".avi"
        results_fname = file_string + 'results_file.spkl'
        save_queue = None

        #un-comment next four lines to enable on-the-fly jpeg conpression and saving to disk
        save_queue = Queue.Queue()
        save_thread_stop_requested = threading.Event()
        save_thread = threading.Thread(target=save_frames, args=(save_queue,save_thread_stop_requested, movie_fname, roi, starting_mode))
        save_thread.start()

        if params.stimulus_mode == 'idle_grating':
            try:
                board = ArduinoMega('/dev/ttyACM0')
                print ("Board recognized.")
            except:
                print "Board cannot be found...exiting."
                sys.exit()
        else:
            pass

        p1,p2,p3 = get_anchor_points(img_cropped)
        fit_queue = Queue.Queue()
        fit_thread_stop_requested = threading.Event()
         # last argument toggles saving of frams. switch off for long trials to avoid out-of-memory errors:
        fit_thread = threading.Thread(target=fit_thread_worker, args=(p1, p2, p3, fit_queue, fit_thread_stop_requested, results_fname, False, False))
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
        cam.close()
        del cam

        print "Assembling results"
        res = Results()
        res.params = params
        #print "Plotting"
        res.plot_alphas(results_fname, file_string)
        print "Saving parameters in .yaml"
        params.save(file_string)
        print "Done, returning."

    if starting_mode !=1 and starting_mode !=2:
        print ("Invalid mode input. Please try again")
    cv2.destroyAllWindows()


'''
import sPickle
import numpy as np

# a = [(i,i**2) for i in xrange(1000000)]
# f = open('lst.spkl', 'w')

for i in range(len(a)):
    sPickle.s_dump_elt(a[i], f)

f.close()




f = open('lst.spkl', 'r')
b = []
for element in sPickle.s_load(open('lst.spkl')):
    b.append(element)

'''
