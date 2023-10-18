import os
import sys
import csv
import rospy
from nmea_msgs.msg import Sentence

directory_path = str
nmea_index_list = ["GGA", "GSV", "GSA", "GLL", "ZDA", "VTG", "RMC"]
nmea_dict = {str : [[]]}
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
                    detail.strip("$")
                    list_tmp.append(detail[:-3])
                    list_tmp.append(detail[:2])
                else:
                    list_tmp.append(detail)

            nmea_dict[index].append()
            

def shutdown_callback():
    # debug 
    print(nmea_dict)
    
    with open(os.path.join(directory_path, "nmea_sentence_table.csv"), "w") as f:
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
            else:
                # Write Header
                csv_writer = csv.writer(f)
                first_row_list = ["NmeaType"]
                first_row_list.extend(ros_message_header_list)
                first_row_list.extend(header_dict[nmea_index])
                csv_writer.writerow(first_row_list)
                 
            if "GSV" in nmea_index:
                for nmea_dict_data_row in nmea_dict[nmea_index]:
                    if len(nmea_dict_data_row) < 16:
                        # Write Detail
                        row_tmp = [nmea_index]
                        row_tmp.extend(nmea_dict_data_row)
                        csv_writer.writerow(row_tmp)
                            
                    elif len(nmea_dict_data_row) < 24:
                        row_tmp = []
                        header = nmea_dict_data_row[:5]
                        row1 = nmea_dict_data_row[6:11]
                        row2 = nmea_dict_data_row[12:17]
                        tail = nmea_dict_data_row[-1:]
                        
                        row_tmp.extend(header)
                        row_tmp.extend(row1)
                        row_tmp.extend(tail)
                        csv_writer.writerow(row_tmp)
                        
                        row_tmp = []
                        row_tmp.extend(header)
                        row_tmp.extend(row2)
                        row_tmp.extend(tail)
                        csv_writer.writerow(row_tmp)
                        
                    else:
                        row_tmp = []
                        header = nmea_dict_data_row[:5]
                        row1 = nmea_dict_data_row[6:11]
                        row2 = nmea_dict_data_row[12:17]
                        row3 = nmea_dict_data_row[18:23]
                        tail = nmea_dict_data_row[-1:]
                        
                        row_tmp.extend(header)
                        row_tmp.extend(row1)
                        row_tmp.extend(tail)
                        csv_writer.writerow(row_tmp)
                        
                        row_tmp = []
                        row_tmp.extend(header)
                        row_tmp.extend(row2)
                        row_tmp.extend(tail)
                        csv_writer.writerow(row_tmp)
                        
                        row_tmp = []
                        row_tmp.extend(header)
                        row_tmp.extend(row3)
                        row_tmp.extend(tail)
                        csv_writer.writerow(row_tmp)             
            else:
                # Write Detail
                for nmea_dict_data_row in nmea_dict[nmea_index]:
                    row_tmp = [nmea_index]
                    row_tmp.extend(nmea_dict_data_row)
                    csv_writer.writerow(row_tmp)
            
if __name__ == '__main__':
    
    rospy.init_node(name="nmea_extraction")
    if len(sys.argv) < 1:
        rospy.loginfo("Need two argument which like: [-d Path to save csv]")
        pass
    
    if '-d' in sys.argv:
        directory_path = sys.argv[sys.argv.index('-d') + 1]
        if not (os.path.exists(directory_path) or os.path.isdir(directory_path)):
            rospy.logerr("Wrong directory path!")
            pass
    
    rospy.Subscriber("nmea_sentence", Sentence, nmea_callback)
    rospy.spin()
    rospy.on_shutdown(shutdown_callback)
    