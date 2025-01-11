#! /usr/bin/python

# import modules
import os, csv, enum, datetime, rclpy
from rclpy.node import Node, Parameter
from nmea_msgs.msg import Sentence
from sensor_msgs.msg import NavSatFix, NavSatStatus
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion

class NMEAData():
    nmea_index_list = ["GGA", "GSV", "GSA", "GLL", "ZDA", "VTG", "RMC"]
    ros_message_header_list = ["Sequence", "FrameID", "Second", "Nanosecond"]

    # URL:https://www.hdlc.jp/~jh8xvh/jp/gnss/nmea.html
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
        
class OdometryData():
    csv_header_list = [
        "TopicName", "Time", "FrameID", "ChildFrameID", 
        "Pose-x", "Pose-y", "Pose-z", "Pose-r", "Pose-p", "Pose-y", "Pose-covariance",
        "Twist-linear-x", "Twist-linear-y", "Twist-linear-z", 
        "Twist-angular-x", "Twist-angular-y", "Twist-angular-z", "Twist-covariance"
    ]

class DataAnalysisNode(Node):
    
    def __init__(self):
        # inheritance the Node
        super().__init__("data_analysis_node")
        self._topic_list_ = self.declare_parameter(
            name="topic_list",
            value=["/gnss/fix", "/ublox/nmea"]
        )
        self._file_path_ = self.declare_parameter(
            name="file_path",
            value=os.path.join(
                os.environ["HOME"], "Documents",
                "rosbag_analysis", "doc"
            )
        )
        
        for topic_name in self._topic_list_.get_parameter_value().string_array_value:
            make_dir_path = os.path.join(self._file_path_.get_parameter_value().string_value, topic_name)
            if os.path.exists(make_dir_path):
                continue
            else:
                os.makedirs(make_dir_path)
        
        self._nmea_count_ = 1

        for topic_name in self._topic_list_.get_parameter_value().string_array_value:
            if "fix" in topic_name:
                self._fix_data_file_ = open(
                    os.path.join(
                        self._file_path_.get_parameter_value().string_value,
                        topic_name,
                        topic_name.replace("/", "_") + "_" + \
                        str(self.get_clock().now().to_msg().sec) + ".csv"
                    ),
                    "w"
                )
                self._fix_csv_writer_ = csv.writer(self._fix_data_file_)
                fix_data = FixData()
                self._fix_csv_writer_.writerow(fix_data.csv_header_list)
                self._fix_sub_ = self.create_subscription(NavSatFix, topic_name, self.fix_callback, qos_profile=10)
                self.get_logger().info("Subscribe to {}".format(topic_name))
                  
            if "gnss/filtered" in topic_name:
                self._fix_filtered_data_file_ = open(
                    os.path.join(
                        self._file_path_.get_parameter_value().string_value,
                        topic_name,
                        topic_name.replace("/", "_") + "_" + \
                        str(self.get_clock().now().to_msg().sec) + ".csv"
                    ),
                    "w"
                )
                self._fix_filtered_csv_writer_ = csv.writer(self._fix_filtered_data_file_)
                fix_data = FixData()
                self._fix_filtered_csv_writer_.writerow(fix_data.csv_header_list)
                self._fix_filtered_sub_ = self.create_subscription(NavSatFix, topic_name, self.fix_filtered_callback, qos_profile=10)
                self.get_logger().info("Subscribe to {}".format(topic_name))

            if "nmea" in topic_name:
                self._nmea_data_file_ = open(
                    os.path.join(
                        self._file_path_.get_parameter_value().string_value,
                        topic_name,
                        topic_name.replace("/", "_") + "_" + \
                        str(self.get_clock().now().to_msg().sec) + ".csv"
                    ),
                    "w"
                )
                self._nmea_csv_writer_ = csv.writer(self._nmea_data_file_)
                nmea_data = NMEAData()
                for nmea_index in nmea_data.nmea_index_list:
                    write_list = nmea_data.ros_message_header_list
                    write_list.extend(nmea_data.header_dict[nmea_index])
                    self._nmea_csv_writer_.writerow([nmea_index])
                    self._nmea_csv_writer_.writerow(write_list)
                
                self._nmea_csv_writer_.writerow([])
                                
                self._nmea_sub_ = self.create_subscription(Sentence, topic_name, self.nmea_callback, qos_profile=10)
                self.get_logger().info("Subscribe to {}".format(topic_name))
                
            if "odometry" in topic_name:
                odometry_data = OdometryData()
                
                if "filtered/global" in topic_name:
                    self._global_odometry_data_file_ = open(
                        os.path.join(
                            self._file_path_.get_parameter_value().string_value,
                            topic_name,
                            topic_name.replace("/", "_") + "_" + \
                            str(self.get_clock().now().to_msg().sec) + ".csv"
                        ),
                        "w"
                    )
                    self._global_odometry_csv_writer_ = csv.writer(self._global_odometry_data_file_)
                    self._global_odometry_csv_writer_.writerow(odometry_data.csv_header_list)
                    self._global_odometry_sub_ = self.create_subscription(Odometry, "odometry/filtered/global", self.global_odom_callback, qos_profile=20)
                    self.get_logger().info("Subscribe to {}".format(topic_name))
                
                if "filtered/local" in topic_name:
                    self._local_odometry_data_file_ = open(
                        os.path.join(
                            self._file_path_.get_parameter_value().string_value,
                            topic_name,
                            topic_name.replace("/", "_") + "_" + \
                            str(self.get_clock().now().to_msg().sec) + ".csv"
                        ),
                        "w"
                    )
                    self._local_odometry_csv_writer_ = csv.writer(self._local_odometry_data_file_)
                    self._local_odometry_csv_writer_.writerow(odometry_data.csv_header_list)
                    self._local_odometry_sub_ = self.create_subscription(Odometry, "odometry/filtered/local", self.local_odom_callback, qos_profile=20)
                    self.get_logger().info("Subscribe to {}".format(topic_name))
                    
                if "gnss" in topic_name:
                    self._gps_odometry_data_file_ = open(
                        os.path.join(
                            self._file_path_.get_parameter_value().string_value,
                            topic_name,
                            topic_name.replace("/", "_") + "_" + \
                            str(self.get_clock().now().to_msg().sec) + ".csv"
                        ),
                        "w"
                    )
                    self._gps_odometry_csv_writer_ = csv.writer(self._gps_odometry_data_file_)
                    self._gps_odometry_csv_writer_.writerow(odometry_data.csv_header_list)
                    self._gps_odometry_sub_ = self.create_subscription(Odometry, "odometry/gnss", self.gps_odom_callback, qos_profile=20)
                    self.get_logger().info("Subscribe to {}".format(topic_name))


    def fix_callback(self, msg : NavSatFix):
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
            'fix', msg.header.frame_id, msg.header.stamp.sec, 
            msg.latitude, msg.longitude, msg.altitude, 
            navsat_status, navsat_service, pos_cov_type, pos_cov_string
        ])
        
        self.get_logger().info("Get Fix Data: {}".format(
            '[' + ', '.join([
                'fix', msg.header.frame_id, str(msg.header.stamp.sec),
                str(msg.latitude), str(msg.longitude), str(msg.altitude)
            ]) + ']'
        ))
        
    def fix_filtered_callback(self, msg : NavSatFix):
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
            'fix_filtered', msg.header.frame_id, msg.header.stamp.sec, 
            msg.latitude, msg.longitude, msg.altitude, 
            navsat_status, navsat_service, pos_cov_type, pos_cov_string
        ])
        
        self.get_logger().info("Get Fix Data: {}".format(
            '[' + ', '.join([
                'fix_filtered', msg.header.frame_id, str(msg.header.stamp.sec),
                str(msg.latitude), str(msg.longitude), str(msg.altitude)
            ]) + ']'
        ))        
    
    def nmea_callback(self, msg : Sentence):
        self.get_logger().info("Get Nmea Sentence: {}".format(msg.sentence))
        
        nmea_data = NMEAData()
        self.get_clock()
        
        write_list = []
        # Header of ROS Message
        write_list.append(self._nmea_count_)
        write_list.append(msg.header.frame_id)
        write_list.append(msg.header.stamp.sec)
        write_list.append(msg.header.stamp.nanosec)
        
        for nmea_detail in msg.sentence[:-3].split(","):
            if "$" in nmea_detail:
                str_tmp = nmea_detail.strip("$")
                write_list.append(str_tmp[:-3])
                write_list.append(str_tmp[2:])
            else:
                write_list.append(nmea_detail)
        
        self._nmea_csv_writer_.writerow(write_list)
        self._nmea_count_ += 1
                
        # debug
        # print(nmea_dict)
        
    def global_odom_callback(self, msg : Odometry):
        stamp = float(msg.header.stamp.sec) + 10**-9 * msg.header.stamp.nanosec
        euler = euler_from_quaternion([
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        ])
        writer_list = [
            "odometry/filtered/global", stamp, msg.header.frame_id, msg.child_frame_id, 
            msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z, euler[0], euler[1], euler[2],
            '[{}]'.format(', '.join(str(msg.pose.covariance))),
            msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z,
            msg.twist.twist.angular.x, msg.twist.twist.angular.y, msg.twist.twist.angular.z,
            '[{}]'.format(', '.join(str(msg.twist.covariance))),
        ]
        
        self._global_odometry_csv_writer_.writerow(writer_list)
        
    def local_odom_callback(self, msg : Odometry):
        stamp = float(msg.header.stamp.sec) + 10**-9 * msg.header.stamp.nanosec
        euler = euler_from_quaternion([
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        ])
        writer_list = [
            "odometry/filtered/local", stamp, msg.header.frame_id, msg.child_frame_id, 
            msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z, euler[0], euler[1], euler[2],
            '[{}]'.format(', '.join(str(msg.pose.covariance))),
            msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z,
            msg.twist.twist.angular.x, msg.twist.twist.angular.y, msg.twist.twist.angular.z,
            '[{}]'.format(', '.join(str(msg.twist.covariance))),
        ]
        
        self._local_odometry_csv_writer_.writerow(writer_list)
    
    def gps_odom_callback(self, msg : Odometry):
        stamp = float(msg.header.stamp.sec) + 10**-9 * msg.header.stamp.nanosec
        euler = euler_from_quaternion([
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        ])
        writer_list = [
            "odometry/gnss", stamp, msg.header.frame_id, msg.child_frame_id, 
            msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z, euler[0], euler[1], euler[2],
            '[{}]'.format(', '.join(str(msg.pose.covariance))),
            msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z,
            msg.twist.twist.angular.x, msg.twist.twist.angular.y, msg.twist.twist.angular.z,
            '[{}]'.format(', '.join(str(msg.twist.covariance))),
        ]
        
        self._gps_odometry_csv_writer_.writerow(writer_list)

def main(args=None):
    rclpy.init(args=args)
    data_analysis_node = DataAnalysisNode()
    rclpy.spin(data_analysis_node)
    rclpy.try_shutdown()

if __name__ == '__main__':
    main()
