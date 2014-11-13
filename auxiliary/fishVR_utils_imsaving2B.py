import roslib
roslib.load_manifest('flyvr')

import numpy as np
import cv2
import yaml
import cPickle as pkl
import streamingpickle as spkl
import matplotlib.pyplot as plt
from pylab import *
import Queue
import flyvr
import time


class Params(object):
    def __init__(self):
        self.contrast = 1.
        self.phase_position = 0.
        self.reset_phase_position = False
        self.orientation = 0.
        self.phase_velocity = 3.1415
        self.wavelength = 0.35
        self.change_offsets = [500,1000,1500,2000]
        self.n_iterations = 1

    def save(self, filename_without_extension):
        with open(filename_without_extension + '.yml', 'w') as outfile:
            outfile.write( yaml.dump(self.__dict__, default_flow_style=False) )

    def load(self, filename):
        stream = file(filename, 'r')
        load_data = yaml.load(stream)
        stream.close()
        for k,v in load_data.items():
            setattr(self, k, v)

    def __str__(self):
        return str(self.__dict__)

class Results(object):
    def __init__(self):
        pass


    def load(self, results_fname):
        b = []
        result_data = open(results_fname, 'rb')
        element = spkl.s_load(result_data).next()
        for element in spkl.s_load(result_data):
            b.append(element)
        timestamps = [belem[0] for belem in b]
        alphas = [belem[1] for belem in b]
        return timestamps, alphas

    def __str__(self):
        return str(self.__dict__)


    def plot_alphas(self, results_fname, save_filename=None):
        alphas, timestamps = Results.load(self, results_fname)

        figure = plt.figure()
        figure.suptitle( '%s' % self.params.stimulus_mode, fontsize=14, fontweight='bold')
        plt.xlabel('timestamps [100 s]')
        plt.ylabel('alpha [degree]')
        ax = figure.add_subplot(111)
        plt.axhline(y=0, color = 'black')
        plt.text(-0.05, 0.33, 'R',
        verticalalignment='bottom', horizontalalignment='left', transform=ax.transAxes,
        color= 'g', fontsize=12)
        plt.text(-0.05, 0.66, 'L',
        verticalalignment='top', horizontalalignment='left', transform=ax.transAxes,
        color= 'g', fontsize=12)
        ymin = -100
        ymax = 100
        plt.ylim((ymin,ymax))
        #for x in np.arange(0, self.timestamps.max(), 500):
            #plt.axvline(x, ymin, ymax, color = 'red')
        ax2 = gca().twinx()
        i_values = np.arange(timestamps[-1])
        if self.params.stimulus_mode == 'bw-fw' or self.params.stimulus_mode == 'left-right':
            phase_traj = np.zeros_like(i_values)
            for i in range(self.params.n_iterations):
                phase_traj[self.params.change_offsets[3]*i:self.params.change_offsets[0]*(i+1)] = self.params.phase_velocity
                phase_traj[self.params.change_offsets[0]*(i+1):self.params.change_offsets[1]*(i+1)] = 0
                phase_traj[self.params.change_offsets[1]*(i+1):self.params.change_offsets[2]*(i+1)] = - self.params.phase_velocity
                phase_traj[self.params.change_offsets[2]*(i+1):self.params.change_offsets[3]*(i+1)] = 0
                self.phase_traj = phase_traj
                ax2.plot(i_values, phase_traj, 'g')
                ax2.set_ylim((phase_traj.min()*1.05,phase_traj.max()*1.05))
                #ax2.fill_between(i_values, 0, phase_traj, facecolor =  '#eeffff', transparent = True)
        if self.params.stimulus_mode == 'fw_acc':
            phase_vel = self.params.phase_velocity_increment_per_10ms * i_values + self.params.phase_velocity
            ax2.plot(i_values, phase_vel, 'g')
        if self.params.stimulus_mode == 'idle_grating':
            trials = ['t3', 't2', 't1', 10,9,8,7,6,5,4,3,2,1,0]
        #black marks intervals (beg. of CS) and red marks of laser
            for x in range(0, int(timestamps[-1])+2, 6000*6): # take * 6 instead of /4 for full 6 min intervals. timestamps.max +02 so that the last line is also drawn.
                plt.axvline(x, ymin, ymax, color = 'black', linewidth = 2. )
            for x in range((6000*6)+300, int(timestamps[-1])-((6000*6)*3), 6000*6): #+300 if the laser sets in after 3s i.e. 300, take * 6 instead of /4 for 6 min, -....*3 for 3 tests
                plt.axvline(x, ymin, ymax, color = 'red', linewidth = 1. )
            for x in range (0, int(timestamps[-1])+2, 6000*6): #timestamps.max +2 so that the last line is also drawn.
                d = trials.pop()
                ds = str (d)
                plt.text(1*x, 0.9, '%s'  %ds, color= 'black')


        if save_filename is not None:
            plt.savefig(save_filename + '.pdf')
        #plt.show()

#-----------this is the ex-matlab tracking code--------
#from profilehooks import profile, coverage

#@profile
def fit_tail(img, p1, p2, p3,
            veclength=200, tail_fit_step_px=30, tail_direction=[-1,0],
            intens_thresh=100):
    '''
    p1 is the starting point for the search. we start walking in tail_direction
    p2 and p3 are the line that defines alpha=0. p2 is the pivot point for alpha evaluation
    '''

    try:
        hor_res = img.shape[0]
        vert_res = img.shape[1]
        intens_thresh = np.percentile(img, 20)
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
        circ = np.zeros((veclength,3), dtype=np.uint16) #kreislaenge=veclength+1=length(a)
        circ[:,0] = points[k-1,0] + r*rr[0]
        circ[:,0] += t*rr[1]
        circ[:,0] = np.round(circ[:,0])
        circ[:,1] = points[k-1,1] + r*rr[1]
        circ[:,1] -= t*rr[0]
        circ[:,1] = np.round(circ[:,1])

        #for i in range(veclength):
        #    if (circ[i][1]<=0) or (vert_res<=circ[i][1]) or (circ[i,0]<=0) or (hor_res<=circ[i,0]):
        #        break
        #    circ[i,2] = img[circ[i,0],circ[i,1]]
        ixs_gteq_zero = np.logical_and(circ[:,0] >= 0, circ[:,1] >= 0)
        ixs_lt_vert_res = circ[:,1] < vert_res
        ixs_lt_hor_res  = circ[:,0] < hor_res
        valid_ixs = np.logical_and(np.logical_and(ixs_gteq_zero, ixs_lt_vert_res), ixs_lt_hor_res)
        circ[valid_ixs, 2] = img[circ[valid_ixs,0], circ[valid_ixs,1]]

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
            circ[:,1] -= t*rr[0]
            circ[:,1] = np.round(circ[:,1])
            #import pdb
            #pdb.set_trace()

            #for i in np.arange(1, veclength):
            #    if (circ[i][1]<0) or (vert_res<=circ[i][1]) or (circ[i,0]<0) or (hor_res<=circ[i,0]):
            #       break
            ixs_gteq_zero = np.logical_and(circ[:,0] >= 0, circ[:,1] >= 0)
            ixs_lt_vert_res = circ[:,1] < vert_res
            ixs_lt_hor_res  = circ[:,0] < hor_res
            valid_ixs = np.logical_and(np.logical_and(ixs_gteq_zero, ixs_lt_vert_res), ixs_lt_hor_res)
            #first_false_ix = np.nonzero(np.logical_not(valid_ixs))[0][0]
            #valid_ixs[first_false_ix:] = False
            circ[valid_ixs, 2] = img[circ[valid_ixs,0], circ[valid_ixs,1]]

            maxval[k-1] = np.max(circ[:,2])
            pos[k-1] = np.argmax(circ[:,2])

            #i = first_false_ix - 1
            #if (maxval[k-2]<intens_thresh or (circ[i,1]<0) or (vert_res<=circ[i,1]) or (circ[i,0]<0) or (hor_res<=circ[i,0])):
            if (maxval[k-2] < intens_thresh):
                break
            points[k,:] = [ circ[pos[k-1],0], circ[pos[k-1],1] ]
            '''
            import matplotlib
            cmap = matplotlib.cm.gray
            cmap.set_bad('r',1.)
            figure()
            img_marked = img.astype(np.float32)
            for ix in np.arange(circ.shape[0]):
                img_marked[circ[ix,0], circ[ix,1]] = np.nan
            for ix in np.arange(points.shape[0]):
                img_marked[points[ix,0], points[ix,1]] = np.nan
            img_marked[points[ix,0], points[ix,1]] = 255
            imshow(img_marked, cmap='gray', interpolation='nearest')
            show(block=False)
            #dt = datetime.datetime.now()
            #file_string = dt.strftime("%Y%m%d_%H%M%S")
            #savefig('fitting_error_plot_' + file_string + '.png')
            '''


        index = np.nonzero(points[:,0])[0]  #index=points(points(:,2)>0); but not indices

        last = np.array(  [ points[index[-1],0], points[index[-1],1] ]  ) #    last=[points(index(end),1),points(index(end),2)];
        a = last - M
        alpha = rad2deg(arccos(np.dot(a,c)/(np.linalg.norm(a)*np.linalg.norm(c))))
        if np.linalg.det(np.vstack((a,c)))<0:     #the matrix must be square!   #if right (a right of c in image, when det <0) neg angle, otherwise (left) pos angle)
            alpha = -alpha
        #alpha = -alpha #to correct left and right angle: left = positive angle, right = negative angle. Remove line if incorrect result!
                                #comment for "fish perspective" as seen by the fish... right and left are switched.

    except Exception,e:
        #print 'WTF: couldnt get alpha!!!'
        #print str(e)
        import traceback
        print traceback.format_exc()
        #print points, maxval, pos
        print circ[valid_ixs,0]
        alpha = np.nan
    print n, k, alpha, intens_thresh
    return alpha, points, circ, pos, maxval

#---------------------------- this is the stimulus and camera recording code------------------------

def get_msg(params):
    ci = flyvr.msg.CylinderGratingInfo()
    ci.contrast = params.contrast
    ci.phase_position = params.phase_position
    ci.reset_phase_position = params.reset_phase_position
    ci.orientation = params.orientation
    ci.phase_velocity = params.phase_velocity
    ci.wavelength = params.wavelength
    return ci


def save_frames(save_queue, stop_requested, filename, roi, starting_mode):
    try:
        print "Saving to file: ", filename
        #code = cv2.cv.CV_FOURCC('X', 'V', 'I', 'D')
        #code = cv2.cv.CV_FOURCC('D', 'I', 'V', 'X')
        #code = cv2.cv.CV_FOURCC('P','I','M','1')
        code = cv2.cv.CV_FOURCC('M','J','P','G')
        writer = None
        if starting_mode == 2:
            writer = cv2.VideoWriter(filename,code,10,(roi[1]-roi[0],roi[3]-roi[2]),True) #how fast the video is played
            if  writer.isOpened() == False:
                print "not open"
                sys.exit()
        if starting_mode == 1:
            cv2.namedWindow('frame')
            cv2.cv.SetMouseCallback('frame', print_coords)

        counter = 0
        keep_going = 1
        while keep_going:
            if save_queue.empty() and stop_requested.is_set():
                print 'wrote everything, finishing up'
                #cv2.destroyAllWindows()
                keep_going = 0
            try:
                fnt = save_queue.get(block=True, timeout=1)
            except Queue.Empty as e:
                continue
            frame,timestamp,caption,vid_observer = fnt

            if starting_mode == 1:
                cv2.namedWindow('frame') #cv2.namedWindow('frame', frame)
                if cv2.waitKey(1) & 0xFF == 27:
                    break
            if starting_mode == 2 and vid_observer == 1:
                if counter%10 == 0:
                    cv2.putText(frame, '%6.2f s' % (timestamp/100.), (20,25), cv2.FONT_HERSHEY_COMPLEX, 0.6, 255, 2)
                    cv2.putText(frame, str(caption), (20,55), cv2.FONT_HERSHEY_COMPLEX, 0.6, 255, 2)
                    frame_color = cv2.cvtColor(frame,cv2.cv.CV_GRAY2RGB)
                    writer.write(frame_color)

            if starting_mode == 1 or starting_mode == 2:
                if counter%10 == 0:
                    cv2.imshow('frame', frame)
                    cv2.waitKey(1)
            counter = counter + 1

    except Exception as e:
        print str(e)
    finally:
        if writer is not None:
            print "Releasing Writer"
            writer.release()
    #cv2.destroyAllWindows()

def print_coords(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        print "x: %5d, y: %5d" % (x,y)

def get_anchor_points(img_cropped):
    '''
    show img and ask user for three points
    first point: start of tail
    second point: pivot point for angle calc
    third point: tail tip
    '''

    fig = figure()
    imshow(img_cropped, cmap='gray') #imshow('video', frame)
    colorbar()
    xlabel('Image array second index')
    ylabel('Image array first index')
    points = ginput(3)
    close(fig)
    p1x, p1y = np.round(points[0])
    p2x, p2y = np.round(points[1])
    p3x, p3y = np.round(points[2])
    return np.array([p1y, p1x]), np.array([p2y, p2x]), np.array([p3y, p3x])


def test_plot():
    res = Results()
    res.load('../fishVR_data/20141112_120446results_file.spkl')
    res.params.stimulus_mode = 'idle_grating'
    res.params.n_iterations = 1
    #res.params.change_offsets = [500,1000,1500,2000]
    res.plot_alphas('20141030new.pdf')


if __name__ == '__main__':
    test_plot()
    #print phase_vel
    #print i_values
