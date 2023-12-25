#! /usr/bin/python

# import modules
import os, csv, enum, datetime
import rclpy, roslib.packages
from rclpy import Node, parameter_service
from nmea_msgs.msg import Sentence
from sensor_msgs.msg import NavSatFix, NavSatStatus

class NMEAData():
    nmea_index_list = ["GGA", "GSV", "GSA", "GLL", "ZDA", "VTG", "RMC"]
    nmea_dict = dict()
    for nmea_index in nmea_index_list:
        nmea_dict[nmea_index] = [[]]
    ros_message_header_list = ["Sequence", "FrameID", "RosStamp"]

    # URL:https://www.hdlc.jp/~jh8xvh/jp/gps/nmea.html
    # URL:https://jp.mathworks.com/help/nav/ref/nmeaparser-system-object.html
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
            "TalkerID", "MessageID", "UTCTime", "FixStatus", 
            "Latitude", "LatitudeDirection",
            "Longitude", "LongtitudeDirection",
            "GroundSpeedKnot", "TrueCourseAngle", "UTCDateTime",
            "MagneticVariation", "MagneticVariationDirection",
            "ModeIndicator", "Validity"
        ]
    }

class FixData():
    csv_header_list = ["Topic", "FrameID", "RosStamp", "Latitude", "Longitude", "Altitude", "NavSatStatus", "SatelliteSystem", "PositionCovarianceType", "PositionCovariance"]

    class Status(enum.Enum):
        STATUS_NO_FIX = NavSatStatus.STATUS_NO_FIX
        STATUS_FIX = NavSatStatus.STATUS_FIX
        STATUS_SBAS_FIX = NavSatStatus.STATUS_SBAS_FIX
        STATUS_GBAS_FIX = NavSatStatus.STATUS_GBAS_FIX

    class Service(enum.Enum):
        SERVICE_GPS = NavSatStatus.SERVICE_GPS
        SERVICE_GLONASS = NavSatStatus.SERVICE_GLONASS
        SERVICE_COMPASS = NavSatStatus.SERVICE_COMPASS
        SERVICE_GALILEO = NavSatStatus.SERVICE_GALILEO

    class PositionCovarianceType(enum.Enum):
        COVARIANCE_TYPE_UNKNOWN = NavSatFix.COVARIANCE_TYPE_UNKNOWN
        COVARIANCE_TYPE_APPROXIMATED = NavSatFix.COVARIANCE_TYPE_APPROXIMATED
        COVARIANCE_TYPE_DIAGONAL_KNOWN = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        COVARIANCE_TYPE_KNOWN = NavSatFix.COVARIANCE_TYPE_KNOWN


class DataAnalysisNode(Node):
    
    def __init__(self):
        # inheritance the Node
        super().__init__("data_analysis_node")
        self._topic_list_ = self.declare_parameter(
            name="topic_list",
            value=[str]
        )
        self._file_path_ = self.declare_parameter(
            name="file_path",
            value=os.path.join(
                roslib.packages.get_pkg_dir("data_analysis"),
                "doc"
            )
        )
        
        for topic_name in self._topic_list_.get_parameter_value().string_array_value:
            if "fix" in topic_name:
                self._fix_sub_ = self.create_subscription(NavSatFix, topic_name, self.fix_subscription)
                self._fix_data_file_ = open(
                    os.path.join(
                        self._file_path_.get_parameter_value().string_value), 
                        "fix", 
                        topic_name.replace("/", "_") + "_{:%Y_%m_%d_%H_%M_%S}.csv".format(datetime.datetime.now()),
                    "w"
                )
                self._fix_csv_writer_ = csv.writer(self._fix_data_file_)
                fix_data = FixData()
                self._fix_csv_writer_.writerow(fix_data.csv_header_list)
                
            if "gps/filtered" in topic_name:
                self._fix_filtered_sub_ = self.create_subscription(NavSatFix, topic_name, self.fix_filtered_subscription)
                self._fix_filtered_data_file_ = open(
                    os.path.join(
                        self._file_path_.get_parameter_value().string_value), 
                        "fix", 
                        topic_name.replace("/", "_") + "_{:%Y_%m_%d_%H_%M_%S}.csv".format(datetime.datetime.now()),
                    "w"
                )
                self._fix_filtered_csv_writer_ = csv.writer(self._fix_filtered_data_file_)
                fix_data = FixData()
                self._fix_csv_writer_.writerow(fix_data.csv_header_list)
                
            if "nmea" in topic_name:
                self._nmea_sub_ = self.create_subscription(Sentence, topic_name, self.nmea_subscription)
                
    def fix_subscription(self, msg = NavSatFix):
        navsat_status = navsat_service = pos_cov_type = ''
        fix_data = FixData()
        
        for item in fix_data.Status:
            if msg.status.status == item.value:
                navsat_status = item.name
        for item in fix_data.Service:
            if msg.status.status == item.value:
                navsat_service = item.name
        for item in fix_data.PositionCovarianceType:
            if msg.position_covariance_type == item.value:
                pos_cov_type = item.name
        
        pos_cov_list = [str(item) for item in msg.position_covariance] 
        pos_cov_row1 = ' ,'.join(pos_cov_list[0:3])
        pos_cov_row2 = ' ,'.join(pos_cov_list[3:6])
        pos_cov_row3 = ' ,'.join(pos_cov_list[6:])
        pos_cov_string = '[[{}], [{}], [{}]]'.format(pos_cov_row1, pos_cov_row2, pos_cov_row3)
                
        self._fix_csv_writer_.writerow([
            'fix', msg.header.frame_id, msg.header.stamp.to_sec(), 
            msg.latitude, msg.longitude, msg.altitude, 
            navsat_status, navsat_service, pos_cov_type, pos_cov_string
        ])
        
        self.get_logger().info("Get Fix Data: {}".format(
            '[' + ', '.join([
                'fix', msg.header.frame_id, str(msg.header.stamp.to_sec()),
                str(msg.latitude), str(msg.longitude), str(msg.altitude)
            ]) + ']'
        ))
        
        
    def fix_filtered_subscription(self, msg = NavSatFix):
        navsat_status = navsat_service = pos_cov_type = ''
        fix_data = FixData()
        
        for item in fix_data.Status:
            if msg.status.status == item.value:
                navsat_status = item.name
        for item in fix_data.Service:
            if msg.status.status == item.value:
                navsat_service = item.name
        for item in fix_data.PositionCovarianceType:
            if msg.position_covariance_type == item.value:
                pos_cov_type = item.name
        
        pos_cov_list = [str(item) for item in msg.position_covariance] 
        pos_cov_row1 = ' ,'.join(pos_cov_list[0:3])
        pos_cov_row2 = ' ,'.join(pos_cov_list[3:6])
        pos_cov_row3 = ' ,'.join(pos_cov_list[6:])
        pos_cov_string = '[[{}], [{}], [{}]]'.format(pos_cov_row1, pos_cov_row2, pos_cov_row3)
                
        self._fix_filtered_csv_writer_.writerow([
            'fix_filtered', msg.header.frame_id, msg.header.stamp.to_sec(), 
            msg.latitude, msg.longitude, msg.altitude, 
            navsat_status, navsat_service, pos_cov_type, pos_cov_string
        ])
        
        self.get_logger().info("Get Fix Data: {}".format(
            '[' + ', '.join([
                'fix_filtered', msg.header.frame_id, str(msg.header.stamp.to_sec()),
                str(msg.latitude), str(msg.longitude), str(msg.altitude)
            ]) + ']'
        ))        
        
    
    def nmea_subscription(self, nmea_msg = Sentence):
        self._nmea_data_ = NMEAData()
        
        self.get_logger().info("Get Nmea Sentence: {}".format(nmea_msg.sentence))
        for index in nmea_data.nmea_index_list:
            if index in nmea_msg.sentence:
                list_tmp = []
                # Header of ROS Message
                list_tmp.append(nmea_msg.header.seq)
                list_tmp.append(nmea_msg.header.frame_id)
                list_tmp.append(nmea_msg.header.stamp.to_sec())
                # Sentence String -> List
                for detail in nmea_msg.sentence[:-3].split(","):
                    if "$" in detail:
                        str_tmp = detail.strip("$")
                        list_tmp.append(str_tmp[:-3])
                        list_tmp.append(str_tmp[2:])
                    else:
                        list_tmp.append(detail)

                nmea_dict[index].append(list_tmp)
                # debug
                # print(nmea_dict)

        

        
        

def main():
    pass

if __name__ == '__main__':
    main()
