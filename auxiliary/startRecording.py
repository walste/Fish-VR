import motmot.cam_iface.cam_iface_ctypes as cam_iface
import cv2
import numpy as nx
import time, sys, os
import motmot.FlyMovieFormat.FlyMovieFormat as FlyMovieFormat
import Queue
import threading
from datetime import datetime
import time


def main():
    
    doit(mode_num=0)

def save_func(writer,save_queue):
    index = 0
    counter = 0
    while 1:
        counter = counter + 1
        fnt = save_queue.get()
        frame,timestamp = fnt
        print timestamp
        cv2.putText(frame,str(timestamp), (20,25), cv2.FONT_HERSHEY_SIMPLEX, 1, 255, 2)  	
        frame = cv2.cvtColor(frame,cv2.cv.CV_GRAY2RGB)  
        writer.write(frame)
        if counter == 100:
            writer.release()
            return
        #if counter > 5:
        #    counter = 0
        #    cv2.imshow('frame',frame)
        #    cv2.waitKey(1)

def doit(device_num=0,
         mode_num=0,
         num_buffers=32,
         ):
    
        
        if mode_num is None:
            mode_num=0
        try:
            cam = cam_iface.Camera(device_num,num_buffers,mode_num)
        except:
            print "device number out of range ... no camera found"
            sys.exit()
            
        
        
        dt = datetime.now()
        fileString = dt.strftime("%Y%m%d_%H%M%S")
        fname = fileString+".avi"
        print "Saving to file: ", fname
        #code = cv2.cv.CV_FOURCC('X', 'V', 'I', 'D')
        #code = cv2.cv.CV_FOURCC('D', 'I', 'V', 'X')
        #code = cv2.cv.CV_FOURCC('P','I','M','1')
        code = cv2.cv.CV_FOURCC('M','J','P','G')
        writer = cv2.VideoWriter(fname,code,25,(2048,2048),True)
        if  writer.isOpened() == False:
            print "not open"
            sys.exit()
        save_queue = Queue.Queue()
        save_thread = threading.Thread( target=save_func, args=(writer,save_queue))
        save_thread.setDaemon(True)
        save_thread.start()
    
        cam.start_camera()
        print "Cam started"

        last_fno = None
        frametick =0
        framecount = 0
        last_fps_print = time.time()

        while True:
            try:
                buf = nx.asarray(cam.grab_next_frame_blocking())
                print 'got frame'
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
            print fno
            if last_fno is not None:
                skip = (fno-last_fno)-1
                if skip != 0:
                    print 'WARNING: skipped %d frames'%skip
                    
            last_fno=fno
            if True:
                save_queue.put( (buf,timestamp) )
            
            now = time.time()
            frametick += 1
            framecount += 1

            t_diff = now-last_fps_print
            if t_diff > 20.0:
                fps = frametick/t_diff
                print "%.1f fps"%fps,
                last_fps_print = now
                frametick = 0
                print " Videosize: " , os.path.getsize(fname)/953674.31641, "MB"


if __name__=='__main__':
    main()
