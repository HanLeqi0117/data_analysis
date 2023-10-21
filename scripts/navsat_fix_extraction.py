#! /usr/bin/python3

import os, csv, enum, datetime
import rospy, roslib.packages
from sensor_msgs.msg import NavSatFix, NavSatStatus

fix_topic = rospy.get_param("fix_topic", default="fix")
fix_filtered_topic = rospy.get_param("fix_filtered_topic", default="gps/filtered")
file_save_path = rospy.get_param("file_save_path", default=os.path.join(roslib.packages.get_pkg_dir(), "doc"))
csv_header_list = ["Topic", "Sequence", "FrameID", "RosStamp", "Latitude", "Longitude", "Altitude", "NavSatStatus", "SatelliteSystem", "PositionCovarianceType", "PositionCovariance"]
fix_seq = 1
fix_filtered_seq = 1
fix_matrix = fix_filtered_matrix = [[]]

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

def fix_callback(msg = NavSatFix):
    for item in Status:
        if msg.status.status == item.value:
            navsat_status = item.name
    for item in Service:
        if msg.status.status == item.value:
            navsat_service = item.name
    for item in PositionCovarianceType:
        if msg.position_covariance_type == item.value:
            pos_cov_type = item.name
    
    pos_cov_list = [str(item) for item in msg.position_covariance] 
    pos_cov_row1 = ' ,'.join(pos_cov_list[0:3])
    pos_cov_row2 = ' ,'.join(pos_cov_list[3:6])
    pos_cov_row3 = ' ,'.join(pos_cov_list[6:])
    pos_cov_string = '[[{}], [{}], [{}]]'.format(pos_cov_row1, pos_cov_row2, pos_cov_row3)
    
    fix_matrix.append([
        fix_topic, fix_seq, msg.header.frame_id, msg.header.stamp.to_sec(), 
        msg.latitude, msg.longitude, msg.altitude, 
        navsat_status, navsat_service, pos_cov_type, pos_cov_string
    ])
    
    fix_seq =+ 1
    

def filtered_fix_callback(msg = NavSatFix):
    for item in Status:
        if msg.status.status == item.value:
            navsat_status = item.name
    for item in Service:
        if msg.status.status == item.value:
            navsat_service = item.name
    for item in PositionCovarianceType:
        if msg.position_covariance_type == item.value:
            pos_cov_type = item.name
    
    pos_cov_list = [str(item) for item in msg.position_covariance] 
    pos_cov_row1 = ' ,'.join(pos_cov_list[0:3])
    pos_cov_row2 = ' ,'.join(pos_cov_list[3:6])
    pos_cov_row3 = ' ,'.join(pos_cov_list[6:])
    pos_cov_string = '[[{}], [{}], [{}]]'.format(pos_cov_row1, pos_cov_row2, pos_cov_row3)
    
    fix_matrix.append([
        fix_topic, fix_filtered_seq, msg.header.frame_id, msg.header.stamp.to_sec(), 
        msg.latitude, msg.longitude, msg.altitude, 
        navsat_status, navsat_service, pos_cov_type, pos_cov_string
    ])
    
    fix_filtered_seq =+ 1

    
def shutdown_callback():
    
    csv_file_name = fix_topic + "_{:%Y_%m_%d_%H_%M_%S}.csv".format(datetime.datetime.now())
    rospy.loginfo("NavSatFix Data is saved in [{}/{}]".format(file_save_path, csv_file_name))

    # Write Fix
    with open(os.path.join(file_save_path, csv_file_name), "w") as f:
        csv_writer = csv.writer(f)
        csv_writer.writerow(csv_header_list)
        for item in fix_matrix:
            csv_writer.writerow(item)

    csv_file_name = fix_filtered_topic + "_{:%Y_%m_%d_%H_%M_%S}.csv".format(datetime.datetime.now())
    rospy.loginfo("NavSatFixFiltered Data is saved in [{}/{}]".format(file_save_path, csv_file_name))
    
    # Write Fix Filtered
    with open(os.path.join(file_save_path, csv_file_name), "w") as f:
        csv_writer = csv.writer(f)
        csv_writer.writerow(csv_header_list)
        for item in fix_filtered_matrix:
            csv_writer.writerow(item)
            
if __name__ == '__main__':
    
    rospy.init_node(name="navsat_fix_extraction")
    rospy.Subscriber(fix_topic, NavSatFix, fix_callback, queue_size=30)
    rospy.Subscriber(fix_filtered_topic, NavSatFix, filtered_fix_callback, queue_size=30)
    rospy.loginfo("Ready to receive NavsatFix and NavsatFixFiltered from Topic.")
    rospy.on_shutdown(shutdown_callback)
