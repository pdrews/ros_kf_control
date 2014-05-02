#!/usr/bin/python
import roslib; roslib.load_manifest('read_bag')
import rosbag
import sys
import os
import cv;
import cv2;
import numpy as np;
import math;
import BagInterface as BI;
import CSVFileIO as IO;
from PIL import ImageFile
import glob;

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


BAGLOC = '/home/paul/1404_bags/out_2.bag'
SAVELOC = '/media/host/1404_2.csv'


def SaveImageFromROSMsg(msg, filename, swap=True):
    cv_img = np_to_cv(rgb_from_pil(pil_from_CompressedImage(msg)));
    if swap :
        cv.CvtColor(cv_img, cv_img, cv.CV_BGR2RGB);
    cv.SaveImage(filename, cv_img);

def pil_from_CompressedImage(msg):
    parser = ImageFile.Parser()
    parser.feed(msg.data)
    res = parser.close()            
    return res 

def np_to_cv(np_array):
    return cv.fromarray(np_array)

def rgb_from_pil(im):
    return np.asarray(im).astype(np.uint8);


def GetAllDataAtOnce(bagfiles, timings, quantities):
    
    qstruct = {};
    for key in quantities.keys() :
        dataseq = BI.InitializeArray(len(timings));
        lastmeasure = (BI.UNKNOWN, timings[0]);
        lidx = 0;
        qstruct[key] = (dataseq, lastmeasure, lidx);
        
    count = 0;
    for bagfile in bagfiles:
        bag = BI.LoadBagFile(bagfile);
        
        for topic, msg, t in bag.read_messages(raw=False):
            
            haskey = None;
            for key in quantities.keys() :
                (topicq, function, interpolate_method, fill_method) = quantities[key];
                if topicq == topic :
                    haskey = key;
                    break;
            
            if haskey is None :
                continue;
            
            time = t.to_sec(); #GetTimeOfMsg(msg);
            
            (topicq, DataReference, interpolate_method, fill_method) = quantities[haskey];
            (dataseq, lastmeasure, lidx) = qstruct[haskey];
            
            if topic == '/axis/image_raw/compressed' :
                data = SAVELOC + repr(count) + '.jpg';
                SaveImageFromROSMsg(msg, data);
                
                if count % 100 == 0 :
                    print 'saving image ' + data
                    print 'image: ' + data + ' at time: ' + repr(time)
                
                count = count + 1;
            else:
                data = DataReference(msg);

            #only use records with data. 
            if data is None :
                continue;
            
            newmeasure = (data, time);
            (lastmeasure, lidx) = BI.UpdateSeq(lastmeasure, newmeasure, dataseq, timings, lidx, interpolate_method);
            qstruct[haskey] = (dataseq, lastmeasure, lidx);
            
        bag.close();
    
    for key in quantities.keys() :
        (dataseq, lastmeasure, lidx) = qstruct[key];
        (topicq, function, interpolate_method, fill_method) = quantities[key];
        
        BI.FillData(dataseq, fill_method);
        qstruct[key] = (dataseq, lastmeasure, lidx);
    
    return qstruct;


def GetAllDataAtOnce2(bagfiles, timings, quantities):
    
    qstruct = {};
    for key in quantities.keys() :
        dataseq = BI.InitializeArray(len(timings));
        lastmeasure = (BI.UNKNOWN, timings[0]);
        lidx = 0;
        qstruct[key] = (dataseq, lastmeasure, lidx);
        
    count = 0;
    count_fv = 0;
    for bagfile in bagfiles:
        bag = BI.LoadBagFile(bagfile);
        
        for topic, msg, t in bag.read_messages(raw=False):
            
            haskey = None;
            for key in quantities.keys() :
                (topicq, function, interpolate_method, fill_method) = quantities[key];
                if topicq == topic :
                    haskey = key;
                    break;
            
            if haskey is None :
                continue;
            
            time = t.to_sec(); #BI.GetTimeOfMsg(msg);
            
            (topicq, DataReference, interpolate_method, fill_method) = quantities[haskey];
            (dataseq, lastmeasure, lidx) = qstruct[haskey];
            
            if topic == '/axis/image_raw/compressed' :
                data = SAVELOC + repr(count) + '.jpg';
#                 SaveImageFromROSMsg(msg, data);
                
                if count % 100 == 0 :
                    print 'saving image ' + data
                    print 'image: ' + data + ' at time: ' + repr(time)
                
                count = count + 1;
            elif topic == '/camera/image_color/compressed':
                data = SAVELOC + repr(count_fv) + '_fv.jpg'
                SaveImageFromROSMsg(msg, data, False);
                
                if count_fv % 100 == 0 :
                    print 'saving image ' + data
                    print 'image: ' + data + ' at time: ' + repr(time)
                
                count_fv = count_fv + 1;
            else:
                data = DataReference(msg);
            
            #only use records with data. 
            if data is None :
                continue;
            
            newmeasure = (data, time);
            (lastmeasure, lidx) = BI.UpdateSeq(lastmeasure, newmeasure, dataseq, timings, lidx, interpolate_method);
            qstruct[haskey] = (dataseq, lastmeasure, lidx);
            
        bag.close();
    
    for key in quantities.keys() :
        (dataseq, lastmeasure, lidx) = qstruct[key];
        (topicq, function, interpolate_method, fill_method) = quantities[key];
        
        BI.FillData(dataseq, fill_method);
        qstruct[key] = (dataseq, lastmeasure, lidx);
    
    return qstruct;


def ExtractImageData(bagfiles, timings, topic) :
    """A generic function definition for reading data from the bag file.
    
    timings is the list of times to extract the data. Because the times
    the messages are received don't align with the timings, an 
    interpolation method also has to be specified. 
    
    DataReference is the function that extracts the desired values from
    the message. 
    """
    
    if len(timings) < 2 :
        raise RuntimeError('The \'Timings\' vector was improperly initialized.')
    
    dataseq = BI.InitializeArray(len(timings));
    
    #interpolation variables
    lastmeasure = (BI.UNKNOWN, timings[0]);
    lidx = 0;
    count = 0;
    for bagfile in bagfiles : 
        print 'bagfile : ' + bagfile
        bag = BI.LoadBagFile(bagfile);
        
        for topic, msg, t in bag.read_messages(topics=topic, raw=False):
            
            data = SAVELOC + repr(count) + '.jpg';
            time = t.to_sec();
            #time = BI.GetTimeOfMsg(msg);
            
            SaveImageFromROSMsg(msg, data);
            if count % 100 == 0:
                print 'saving image ' + data
                print 'image: ' + data + ' at time: ' + repr(time)
            
            
            #if the time isn't given as part of the message, it's buried inside the message.
            #assume the data reference includes a definition for the time.
            if time is None :
                raise RuntimeException('The time of the message is unknown')
            
            newmeasure = (data, time);
            (lastmeasure, lidx) = BI.UpdateSeq(lastmeasure, newmeasure, dataseq, timings, lidx, BI.INTERPOLATE_CLOSEST);
            count = count+1;

        bag.close();
    
    BI.FillData(dataseq, BI.FILL_ALL);
    
    return dataseq;


def DealWithLIDAR(msg):
    min = 100000000000000000000.0;
    max = -100000000000.0;
    count = 0;
    sum = 0.0;
    for i in range(0, len(msg.ranges)) :
        cur = msg.ranges[i];

        if cur > msg.range_min + 0.2 and cur < msg.range_max - 0.2 :
            count = count + 1;
            sum = sum + cur;
            if cur > max :
                max = cur;
            if cur < min :
                min = cur;
    
    avg = sum/len(msg.ranges);
    proper_avg = sum/count;
    return (count, len(msg.ranges), min, max, avg, proper_avg);
    

def GetQuantitiesFromBagfiles(bagfiles) : 
    quantity = {};
    data = {};
    
    quantity['laser_odom_pose_x_error,laser_odom_pose_y_error,laser_odom_orient_x_error,laser_odom_orient_y_error,laser_odom_orient_z_error,laser_odom_orient_w_error'] = ('/icp_error_odom', lambda msg: (msg.pose.pose.position.x,msg.pose.pose.position.x,msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w), BI.INTERPOLATE_CLOSEST, BI.FILL_ALL);
    quantity['pose_x,pose_y,orient_x,orient_y,orient_z,orient_w'] = ('/icp_odom', lambda msg: (msg.pose.pose.position.x,msg.pose.pose.position.y,msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w), BI.INTERPOLATE_WEIGHTED, BI.FILL_ALL);
    quantity['gps_odom_x,gps_odom_y'] = ('/gps/odom', lambda msg: (msg.pose.pose.position.x,msg.pose.pose.position.y), BI.INTERPOLATE_CLOSEST, BI.FILL_ALL);
    quantity['gps_vel_x,gps_vel_y'] = ('/gps/vel', lambda msg: (msg.twist.linear.x,msg.twist.linear.y), BI.INTERPOLATE_CLOSEST, BI.FILL_ALL);
    quantity['compass_compass,compass_heading,compass_heading_rate,compass_stddev'] = ('/compass/compass', lambda msg: (msg.compass,msg.heading,msg.heading_rate,msg.stddev), BI.INTERPOLATE_CLOSEST, BI.FILL_ALL);
    quantity['imu_rate,imu_x,imu_y,imu_z'] = ('/imu/data', lambda msg: (msg.angular_velocity.z,msg.linear_acceleration.x,msg.linear_acceleration.y,msg.linear_acceleration.z), BI.INTERPOLATE_CLOSEST, BI.FILL_ALL);
    quantity['ldrive,rdrive'] = ('/cmd_drive', lambda msg: (msg.left,msg.right), BI.INTERPOLATE_CLOSEST, BI.FILL_ALL);
    quantity['linear,angular'] = ('/cmd_vel', lambda msg: (msg.linear.x,msg.angular.z), BI.INTERPOLATE_CLOSEST, BI.FILL_ALL);

    
    data['time'] = GetTimeSequence(bagfiles, '/icp_odom', 0.05);
#     data['image times'] = ExtractImageData(bagfiles, data['timings'], '/axis/image_raw/compressed');
    
#     for key in quantity.keys() :
#         print 'getting data for ' + key
#         (topic, function, interpolation_method, fill_method) = quantity[key];
#         data[key] = BI.GetDataFromBagFiles(bagfiles, data['timings'], topic, function, interpolation_method, fill_method);
    
    qstruct = GetAllDataAtOnce2(bagfiles, data['time'], quantity);
    for key in qstruct.keys() :
        (qdata, lastm, lidx) = qstruct[key];
        data[key] = qdata;
    
    return data;


def GetBagFileNames(directory):
    bagfiles = glob.glob(directory + '*.bag');
    bagfiles.sort();
    if len(bagfiles) == 0 :
        raise RuntimeError('files are not included in ' + directory + '*.bag');
    return bagfiles


def CheckArgs(argv) :
    if len(argv) < 3:
        raise RuntimeError('arguments: <bagfile> <saveloc>');


#load in multiple bag files (done)
#get the time of the first entry in the first bag file and the last entry in the last bag file.
#create a timings array using this.
#the fill is different

def GetFirstTime(bag, topic):
    for topic, msg, t in bag.read_messages(topic, raw=False):
        return BI.GetTimeOfMsg(msg); #t.to_sec();#
    
    
def GetLastTime(bag, topic):
    last = BI.UNKNOWN;
    for topic, msg, t in bag.read_messages(topic, raw=False):
        last = BI.GetTimeOfMsg(msg); #t.to_sec();#
    return last;


def GetTimeSequence(bagfiles, topic, increment):
    seq = [];
    first = GetFirstTime(BI.LoadBagFile(bagfiles[0]), topic);
    last = GetLastTime(BI.LoadBagFile(bagfiles[len(bagfiles)-1]), topic);

    print 'first: ' + repr(first) + ', from file: ' + bagfiles[0];
    print 'last: ' + repr(last) + ', from file: ' + bagfiles[len(bagfiles)-1];
    

    #the format of these numbers may be in ms or micro seconds.
    offset = first;
    while offset < last :
        seq.append(offset);
        offset = offset + increment;
    return seq;


def program0():
    #load the bag file
    print 'Loading the bag file...'
    bag = BI.LoadBagFile(name);#sys.argv[1]);

    #save the data
    IO.SaveDataAsCSV(data, SAVELOC + sys.argv[2]);
    
    bag.close();
    

#program to read the data from all the bag files in the specified folder.
def main(argv=None) : 
    """Program to extract time series data from bag files. 
    """

    print 'Starting processing'
    
    if argv is None : 
        argv = sys.argv
    try : 
        #CheckArgs(sys.argv);
        
        #name = BAGLOC + '130818_around_the_lake_auto/' + '2013-08-18-08-09-47_0.bag'
        print "1"
        
        bagfiles = [BAGLOC] #GetBagFileNames(BAGLOC + sys.argv[1] + '/')
        print "2"

        #get the time series data
        data = GetQuantitiesFromBagfiles(bagfiles);
        print "3"
        #save the data
        IO.SaveDataAsCSV(data, SAVELOC);
        print "4"
        
    except IOError as err:
        print 'Cannot load the bag file: ' + err.strerror
        return 2
    except RuntimeError as err:
        print err
        return 2
    
if __name__ == "__main__":
    sys.exit(main())
