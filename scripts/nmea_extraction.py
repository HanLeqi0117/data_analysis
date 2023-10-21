#! /usr/bin/python3

import os
import sys
import csv
import rospy
import roslib.packages
from datetime import datetime
from nmea_msgs.msg import Sentence

file_save_path = rospy.get_param("file_save_path", default=os.path.join(roslib.packages.get_pkg_dir(), "doc"))
recieve_topic_name = rospy.get_param("recieve_topic_name", default="nmea_sentence")
nmea_index_list = ["GGA", "GSV", "GSA", "GLL", "ZDA", "VTG", "RMC"]
nmea_dict = dict()
for nmea_index in nmea_index_list:
    nmea_dict[nmea_index] = [[]]
ros_message_header_list = ["Sequence", "FrameID", "RosStamp"]

# 参考URL:https://www.hdlc.jp/~jh8xvh/jp/gps/nmea.html
# 参考URL:https://jp.mathworks.com/help/nav/ref/nmeaparser-system-object.html
header_dict = {
    "GGA":[
        "TalkerID", "MessageID", "UTCTime", 
        "Latitude", "LatitudeDirection",
        "Longitude", "LongtitudeDirection", 
        "QualityIndicator", "NumSatellitesInUse", 
        "HDOP", "Altitude", "AltitudeUnit",
        "GeoidSeparation", "GeoidUnit", 
        "AgeOfDifferentialData", "DifferentialReferenceStationID"
    ],
    "GSV":[
        "TalkerID", "MessageID", "NumSentences", 
        "SentenceNumber", "SatellitesInView",
        "SatelliteID", "Elevation", 
        "Azimuth", "SNR", "SignalID"
    ],
    "GSA":[
        "TalkerID", "MessageID", "Mode", 
        "FixType", "SatellitesIDNumber",
        "PDOP", "VDOP", "HDOP", "SystemID"
    ],
    "GLL":[
        "TalkerID", "MessageID", 
        "Latitude", "LatitudeDirection",
        "Longitude", "LongtitudeDirection", 
        "UTCTime", "DataValidity", "PositioningMode", "DOP"
    ],
    "ZDA":[
        "TalkerID", "MessageID", "UTCTime", 
        "UTCDay", "UTCMonth", "UTCYear",
        "LocalZoneHours", "LocalZoneMinutes"
    ],
    "VTG":[
        "TalkerID", "MessageID", 
        "TrueCourseAngle", "TureCourse", 
        "MagneticCourseAngle", "MagneticCourse",
        "GroundSpeedKnot", "Knot", 
        "GroundSpeedKmh", "Kmh", 
        "ModeIndicator"
    ],
    "RMC":[
        "TalkerID", "MessageID", "FixStatus", 
        "Latitude", "LatitudeDirection",
        "Longitude", "LongtitudeDirection",
        "GroundSpeedKnot", "TrueCourseAngle", "UTCDateTime",
        "MagneticVariation", "MagneticVariationDirection",
        "ModeIndicator"
    ]
}

def nmea_callback(msg = Sentence):
    rospy.loginfo("Get Nmea Sentence: {}".format(msg.sentence))
    for index in nmea_index_list:
        if index in msg.sentence:
            list_tmp = []
            # Header of ROS Message
            list_tmp.append(msg.header.seq)
            list_tmp.append(msg.header.frame_id)
            list_tmp.append(msg.header.stamp.to_sec())
            # Sentence String -> List
            for detail in msg.sentence[:-3].split(","):
                if "$" in detail:
                    str_tmp = detail.strip("$")
                    list_tmp.append(str_tmp[:-3])
                    list_tmp.append(str_tmp[2:])
                else:
                    list_tmp.append(detail)

            nmea_dict[index].append(list_tmp)
            # debug
            # print(nmea_dict)
            
def shutdown_callback():
    # debug 
    # print(nmea_dict)
    
    csv_file_name = recieve_topic_name + "_{:%Y_%m_%d_%H_%M_%S}.csv".format(datetime.now())
    rospy.loginfo("NmeaSentence Data is saved in [{}/{}]".format(file_save_path, csv_file_name))
        
    with open(os.path.join(file_save_path, csv_file_name), "w") as f:
        for nmea_index in nmea_index_list:
            if len(nmea_dict[nmea_index]) == 1 and len(nmea_dict[nmea_index][0]) == 0:
                continue
            
            if "GSA" in nmea_index:
                # Write Header
                csv_writer = csv.writer(f)
                first_row_list = ["NmeaType"]
                first_row_list.extend(ros_message_header_list)
                for index in header_dict[nmea_index]:
                    if "SatellitesIDNumber" in index:
                        first_row_list.append('SatellitesIDNumber')
                        for i in range(11):
                            first_row_list.append('')
                    else:
                        first_row_list.append(index)
                csv_writer.writerow(first_row_list)
            else:
                # Write Header
                csv_writer = csv.writer(f)
                first_row_list = ["NmeaType"]
                first_row_list.extend(ros_message_header_list)
                first_row_list.extend(header_dict[nmea_index])
                csv_writer.writerow(first_row_list)
                
            if "GSV" in nmea_index:
                for nmea_dict_data_row in nmea_dict[nmea_index]:                
                    if len(nmea_dict_data_row) == 17:
                        header = nmea_dict_data_row[:9]
                        row1 = nmea_dict_data_row[9:13]
                        row2 = nmea_dict_data_row[13:17]
                        tail = nmea_dict_data_row[-1:]
                        
                        row_tmp = [nmea_index]
                        row_tmp.extend(header)
                        row_tmp.extend(row1)
                        row_tmp.extend(tail)
                        csv_writer.writerow(row_tmp)
                        
                        row_tmp = [nmea_index]
                        row_tmp.extend(header)
                        row_tmp.extend(row2)
                        row_tmp.extend(tail)
                        csv_writer.writerow(row_tmp)
                        
                    elif len(nmea_dict_data_row) == 25:
                        header = nmea_dict_data_row[:9]
                        row1 = nmea_dict_data_row[9:13]
                        row2 = nmea_dict_data_row[13:17]
                        row3 = nmea_dict_data_row[17:21]
                        row4 = nmea_dict_data_row[21:25]
                        tail = nmea_dict_data_row[-1:]
                        
                        row_tmp = [nmea_index]
                        row_tmp.extend(header)
                        row_tmp.extend(row1)
                        row_tmp.extend(tail)
                        csv_writer.writerow(row_tmp)
                        
                        row_tmp = [nmea_index]
                        row_tmp.extend(header)
                        row_tmp.extend(row2)
                        row_tmp.extend(tail)
                        csv_writer.writerow(row_tmp)
                        
                        row_tmp = [nmea_index]
                        row_tmp.extend(header)
                        row_tmp.extend(row3)
                        row_tmp.extend(tail)
                        csv_writer.writerow(row_tmp)
                        
                        row_tmp = [nmea_index]
                        row_tmp.extend(header)
                        row_tmp.extend(row4)
                        row_tmp.extend(tail)
                        csv_writer.writerow(row_tmp)
                                
            else:
                # Write Detail
                for nmea_dict_data_row in nmea_dict[nmea_index]:
                    if len(nmea_dict_data_row) == 0:
                        continue
                    row_tmp = [nmea_index]
                    row_tmp.extend(nmea_dict_data_row)
                    csv_writer.writerow(row_tmp)

            
if __name__ == '__main__':
    
    rospy.init_node(name="nmea_extraction")
    rospy.Subscriber(recieve_topic_name, Sentence, nmea_callback, queue_size=30)
    rospy.loginfo("Ready to receive NmeaSentence from Topic.")
    rospy.spin()
    rospy.on_shutdown(shutdown_callback)
    