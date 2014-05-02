#!/usr/bin/env python
import sys
import os
import yaml;
import string

def _check_float(mystring):
    """Return true if the number looks like a float
    """
    return not set(mystring) - set(string.digits) - set('.-e');

def _check_int(mystring):
    """Return true if the number looks like an integer
    """
    return not set(mystring) - set(string.digits) - set('-')

def _check_str(mystring):
    """Return true if the field contains a string
    """
    return len(mystring) > 2 and mystring[0] == '\'' and mystring[len(mystring)-1] == '\'';

def _GetData(string):
    """
    Converts the data from string to some format, depending on the characters in the data.
    """
    
    if len(string) <= 0 :
        #how to know whether it's a string, float, int, NoneType, etc.
        return None;
    elif string == 'None' :
        return None;
    elif string == 'False' :
        return False;
    elif string == 'True' :
        return True;
    elif string == 'inf' or string == '-inf' :
        return float(string);
    elif string[0] == '[' and string[len(string)-1] == ']' :
        array_data = string[1:len(string)-1];
        array_data = array_data.rsplit(';');
        arr = [];
        for i in range(0, len(array_data)) :
            if _check_float(array_data[i]) :
                arr.append(float(array_data[i]));
            elif _check_int(array_data[i]) :
                arr.append(int(array_data[i]));
            else :
                print 'CSVFileIO: Ambiguous array data type: ' + array_data[i]
                arr.append(array_data[i])
        return arr
    elif _check_int(string) :
        return int(string);
    elif _check_float(string) :
        return float(string);
    elif _check_str(string) :
        return string[1:len(string)-1];
    else :
        pr = 'CSVFileIO: Ambiguous data type:' + string;
        pr = '\n' + '(if the data has quotes surrounding it, it probably means you opened the .csv file in excel and then saved it.)'
        raise RuntimeError(pr);


def LoadXY(filename) :
    """Loads only vectors named 'X' and 'Y'.
    Used for loading curves, or car positions, for example.
    """
    f = open(filename, 'r');
    
    x = [];
    y = [];
    
    keys = {};
    mapping = [];
    bfirstline = True;
    for line in f :
        line = line.strip();
        entries = line.rsplit(',');
        if bfirstline : 
            for i in range(0, len(entries)) :
                mapping.append(entries[i]);
            bfirstline = False;
        else :
            for i in range(0, len(entries)) :
                if mapping[i] == 'X' :
                    x.append(_GetData(entries[i]))
                elif mapping[i] == 'Y' :
                    y.append(_GetData(entries[i]))
    f.close();
    return (x, y);


def LoadDataAsCSV(filename):
    """load in the data using the same format as was used to save the data
    """
    f = open(filename, 'r');
    
    mapping = [];
    data = {};
    count = 0;
    for line in f :
        line = line.strip();
        entries = line.rsplit(',');
        if count == 0 :
            for key in entries :
                mapping.append(key)
                data[key] = [];
        else :
            for j in range(0, len(entries)) :
                key = mapping[j];
                data[key].append(_GetData(entries[j]));
        count = count + 1;
    
    if data.has_key('entry') :
        data.pop('entry')
    
    f.close();
    
    return data;


def SaveDataAsCSV(data, filename):
    """
    Takes in a dict and a string as arguments. Outputs the data to the file in csv format.
    
    The top line in the csv file lists the keys of the dictionary. The rest of the data 
    corresponds to the data in the dictionary.
    """
    
    entry = 0;
    try : 
        f = open(filename, 'w');
        
        #print header names
        str = 'entry'
        for key in data.keys() :
            str = str + ',' + key
        str = str + '\n'
        f.write(str);
        
        #print the data
        arr = data.keys()
        print 'putting the stuff in the csv file'
        for i in range(0, len(data[arr[0]])) :
            str = '' + repr(entry);
            for key in arr :
                if not data[key] is None :
                    if not i < len(data[key]) :
                        #if the vector isn't long enough, assume it's None
                        if i == 0:
                            print key + ' is too short'
                        str = str + ',None';
                    elif not (type(data[key][i]) is list or type(data[key][i]) is tuple) :
                        str = str + ',' + repr(data[key][i]);
                    else : 
                        ls = ''
                        for j in range(0, len(data[key][i])) :
                            if j < len(data[key][i]) - 1 :
                                ls = ls + repr(data[key][i][j]) + ','
                            else :
                                ls = ls + repr(data[key][i][j]) 
                        str = str + ',' + ls
                else : 
                    str = str + ',None';
                    if i == 0 :
                        print key + ' is NoneType'
            str = str + '\n'
            f.write(str);
            entry = entry + 1;
        f.close();

    except IOError as err:
        print 'Cannot write to the file. ' + err.strerror
        return 2








