#!/usr/bin/env python
import roslib; roslib.load_manifest('read_bag')
import rosbag
import sys
import os

INTERPOLATE_WEIGHTED = 0;
INTERPOLATE_CLOSEST = 1;
INTERPOLATE_LAST_KNOWN = 2;
INTERPOLATE_MOST_RECENT = 3;

FILL_NONE = 0
FILL_ALL = 1
FILL_END = 2

UNKNOWN = None #used as a filler for lack of data.


###################################################################################################################################################################
################################################################ Code to extract data from the bag file ###########################################################
###################################################################################################################################################################

def LoadBagFile(bagfile) :
    """Abstract the function for loading the bag file.
    """
    return rosbag.Bag(bagfile);


def FillData(dataseq, fill_method):
    """Fills in gaps before and after the data starts.
    
    fill_method can be FILL_NONE, FILL_ALL, or FILL_END.
    
    FILL_ALL fills in missing data at the beginning and end.
    FILL_END only fills in missing data at the end.
    """
    if fill_method == FILL_NONE:
        return
    else :
        # FILL_END : the default
        first_entry_idx = UNKNOWN;
        last_entry_val = UNKNOWN;
        for i in range(0, len(dataseq)) :
            if dataseq[i] is UNKNOWN :
                dataseq[i] = last_entry_val;
            else :
                last_entry_val = dataseq[i];
                if first_entry_idx is UNKNOWN :
                    first_entry_idx = i;
        if fill_method == FILL_ALL and not first_entry_idx is None:
            for i in range(0, first_entry_idx) :
                dataseq[i] = dataseq[first_entry_idx];


def InterpolateData(lastdata, newdata, t_star, interpolate_method) :
    """Interpolating between lastdata and newdata using t_star to weight between them.
    
    Interpolate_method specifies how to do the interpolation. 
    """
    (m1, tl) = lastdata;
    (m2, tn) = newdata;
    if interpolate_method == INTERPOLATE_WEIGHTED :        
        w2 = (t_star - tl)/(tn-tl);
        w1 = (tn - t_star)/(tn-tl);
        if type(m1) is list or type(m1) is tuple:
            return [w1*m1[i] + w2*m2[i] for i in range(0,len(m1))]; #used for vector data; velocity
        else :
            return w1*m1 + w2*m2
    elif interpolate_method == INTERPOLATE_CLOSEST :
        closest = m1;
        if abs(t_star - tl) > abs(t_star - tn) :
            closest = m2;
        if closest is UNKNOWN :
            if not m1 is UNKNOWN:
                return m1;
            else :
                return m2;
        return closest;
    elif interpolate_method == INTERPOLATE_LAST_KNOWN :
        if m1 is UNKNOWN :
            (m2, t2) = newdata;
            return m2;
        return m1;
    elif interpolate_method == INTERPOLATE_MOST_RECENT :
        if m2 is UNKNOWN :
            (m1, t1) = lastdata;
            return m1;
        return m2;
    else :
        raise RuntimeError('Not a valid interpolate method. ' + repr(interpolate_method))


def UpdateSeq(lastdata, newdata, seq, timings, lidx, interpolate_method=0) :
    """Figures out what data points to update, and updates them. 
    
    Updates all the values indicated by timings between lastdata and newdata.
    """
    (m1, tl) = lastdata
    if interpolate_method > 0 or m1 != UNKNOWN :
        (_, tn) = newdata

        while lidx < len(timings) and timings[lidx] <= tn:
            if timings[lidx] >= tl :
                seq[lidx] = InterpolateData(lastdata, newdata, timings[lidx], interpolate_method);
            lidx = lidx + 1;

    return (newdata, lidx);


def GetDataFromBagFile(bag, timings, topic, DataReference, interpolate_method=INTERPOLATE_WEIGHTED, fill_method=FILL_NONE) :
    """A generic function definition for reading data from the bag file.
    
    timings is the list of times to extract the data. Because the times
    the messages are received don't align with the timings, an 
    interpolation method also has to be specified. 
    
    DataReference is the function that extracts the desired values from
    the message. 
    """
    if len(timings) < 2 :
        raise RuntimeError('The \'Timings\' vector was improperly initialized.')
        
    dataseq = InitializeArray(len(timings));
    
    #interpolation variables
    lastmeasure = (UNKNOWN, timings[0]);
    lidx = 0;
    for topic, msg, t in bag.read_messages(topics=topic, raw=False):
        
        data = DataReference(msg);
        time = GetTimeOfMsg(msg);
        
        #treat this message like none was received if the data isn't what's desired.
        if data is None :
            continue;
        
        #if the time isn't given as part of the message, it's buried inside the message.
        #assume the data reference includes a definition for the time. 
        if time is None :
            if type(data) is list or type(data) is tuple :
                time = data[1];
                data = data[0];
            else :
                GetTimeOfMsg(t); #use this less accurate time as a last resort.
                print 'Warning: the msg time is unspecified. Using the bag file time. topic: ' + topic 

        newmeasure = (data, time);
        (lastmeasure, lidx) = UpdateSeq(lastmeasure, newmeasure, dataseq, timings, lidx, interpolate_method);
    
    FillData(dataseq, fill_method);
    
    return dataseq;


def GetDataFromBagFiles(bagfiles, timings, topic, DataReference, interpolate_method=INTERPOLATE_WEIGHTED, fill_method=FILL_NONE) :
    """A generic function definition for reading data from the bag file.
    
    timings is the list of times to extract the data. Because the times
    the messages are received don't align with the timings, an 
    interpolation method also has to be specified. 
    
    DataReference is the function that extracts the desired values from
    the message. 
    """
    if len(timings) < 2 :
        raise RuntimeError('The \'Timings\' vector was improperly initialized.')
    
    dataseq = InitializeArray(len(timings));
    
    #interpolation variables
    lastmeasure = (UNKNOWN, timings[0]);
    lidx = 0;
    for bagfile in bagfiles:
        bag = LoadBagFile(bagfile);
        
        for topic, msg, t in bag.read_messages(topics=topic, raw=False):
            
            data = DataReference(msg);
            time = GetTimeOfMsg(msg);
            
            #only use records with data. 
            if data is None :
                continue;
            
            if time is None :
                time = t.to_sec();
            
            newmeasure = (data, time);
            (lastmeasure, lidx) = UpdateSeq(lastmeasure, newmeasure, dataseq, timings, lidx, interpolate_method);
            
        bag.close();
    
    FillData(dataseq, fill_method);
    
    return dataseq;


def GetAllDataAtOnce(bagfiles, timings, quantities):
    
    qstruct = {};
    for key in quantities.keys() :
        dataseq = InitializeArray(len(timings));
        lastmeasure = (UNKNOWN, timings[0]);
        lidx = 0;
        qstruct[key] = (dataseq, lastmeasure, lidx);
        
    
    for bagfile in bagfiles:
        bag = LoadBagFile(bagfile);
        
        for topic, msg, t in bag.read_messages(raw=False):
            
            haskey = None;
            for key in quantities.keys() :
                (topicq, function, interpolate_method, fill_method) = quantities[key];
                if topicq == topic :
                    haskey = key;
                    break;
            
            if haskey is None :
                continue;
            
            (topicq, DataReference, interpolate_method, fill_method) = quantities[haskey];
            (dataseq, lastmeasure, lidx) = qstruct[haskey];
            
            data = DataReference(msg);
            time = GetTimeOfMsg(msg);
            
            #only use records with data. 
            if data is None :
                continue;
            
            if time is None :
                time = t.to_sec();
            
            newmeasure = (data, time);
            (lastmeasure, lidx) = UpdateSeq(lastmeasure, newmeasure, dataseq, timings, lidx, interpolate_method);
            qstruct[haskey] = (dataseq, lastmeasure, lidx);
            
        bag.close();
    
    for key in quantities.keys() :
        (dataseq, lastmeasure, lidx) = qstruct[key];
        (topicq, function, interpolate_method, fill_method) = quantities[key];
        
        FillData(dataseq, fill_method);
        qstruct[key] = (dataseq, lastmeasure, lidx);
    
    return qstruct;
    


def GetTimeOfMsg(msg):
    """Extracts a float representing the time from the msg.
    """
    if not hasattr(msg, 'header') :
        return None;
    
    return msg.header.stamp.secs + 1.0*msg.header.stamp.nsecs/1000000000.0;


def InitializeArray(n):
    """Initializes an array with n entries to have value UNKNOWN.
    """
    dataseq = [];
    for i in range(0, n) : 
        dataseq.append(UNKNOWN);
    return dataseq;


def GetTimeSequence(bag, topic, increment):
    """Creates a list of timing data for the bag file. 
    
    The start time is the first time the topic message is recorded. 
    Entries are added in increments of 0.1 until the end time is greater
    than the last time the topic message is recorded.
    
    This is used for interpolating values of other data in the bag file
    so they all line up to the same timings.
    """
    seq = [];
    first = UNKNOWN;
    last = UNKNOWN;
    for topic, msg, t in bag.read_messages(raw=False):
        if first is UNKNOWN :
            first = t.to_sec(); #GetTimeOfMsg(msg);
        if not last is UNKNOWN and last < t.to_sec() :
            last = t.to_sec(); #GetTimeOfMsg(msg);

    #the format of these numbers may be in ms or micro seconds.
    offset = first;
    while offset < last :
        seq.append(offset);
        offset = offset + increment;
    return seq;


