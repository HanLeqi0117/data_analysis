#! /usr/bin/python3

import os
import sys
import csv
import rospy
from nmea_msgs.msg import Sentence

change_file_flag = False
directory_path = str()
csv_file_name = str()
nmea_index_list = ["GGA", "GSV", "GSA", "GLL", "ZDA", "VTG", "RMC"]
nmea_dict = dict()
for nmea_index in nmea_index_list:
    nmea_dict[nmea_index] = [[]]
ros_message_header_list = ["Sequence", "FrameID", "RosSec", "RosNsec"]

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
            list_tmp.append(msg.header.stamp.secs)
            list_tmp.append(msg.header.stamp.nsecs)
            list_tmp.append(msg.header.frame_id)
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
    if change_file_flag:
        rospy.loginfo("NmeaSentence Data is saved in [{}/{}]".format(directory_path, csv_file_name))
        
        with open(os.path.join(directory_path, csv_file_name), "w") as f:
            for nmea_index in nmea_index_list:
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
                        if len(nmea_dict_data_row) == 18:
                            header = nmea_dict_data_row[:8]
                            row1 = nmea_dict_data_row[9:12]
                            row2 = nmea_dict_data_row[13:16]
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
                            
                        elif len(nmea_dict_data_row) == 26:
                            header = nmea_dict_data_row[:8]
                            row1 = nmea_dict_data_row[9:12]
                            row2 = nmea_dict_data_row[13:16]
                            row3 = nmea_dict_data_row[17:20]
                            row4 = nmea_dict_data_row[21:24]
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
                        row_tmp = [nmea_index]
                        row_tmp.extend(nmea_dict_data_row)
                        csv_writer.writerow(row_tmp)
    else:
        rospy.loginfo("No change happened.")
            
if __name__ == '__main__':
    
    rospy.init_node(name="nmea_extraction")
    if len(sys.argv) < 4:
        rospy.loginfo("usage: [-d Path to save csv] [-f CSV file name]")
        sys.exit()
    
    if '-d' in sys.argv:
        directory_path = sys.argv[sys.argv.index('-d') + 1]
        if not (os.path.exists(directory_path) or os.path.isdir(directory_path)):
            rospy.logwarn("Directory given is not existed!")
            sys.exit()
    else:
        rospy.loginfo("-d option need!")
    
    if '-f' in sys.argv:
        csv_file_name = sys.argv[sys.argv.index('-f') + 1]
        if csv_file_name in os.listdir(directory_path):
            if not ".csv" in csv_file_name:
                csv_file_name = csv_file_name + ".csv"
            rospy.logwarn("File name given is existed: {}".format(csv_file_name))
            sys.exit()
        elif csv_file_name == '':
            csv_file_name = "nmea_sentence_table.csv"
            rospy.logwarn("File name is empty, change it to default: {}".format(csv_file_name))
            sys.exit()
    else:
        rospy.loginfo("-f option need!")
        sys.exit()

    change_file_flag = True
    rospy.Subscriber("nmea_sentence", Sentence, nmea_callback, queue_size=30)
    rospy.loginfo("Ready to receive NmeaSentence from Topic.")
    rospy.spin()
    rospy.on_shutdown(shutdown_callback)
    