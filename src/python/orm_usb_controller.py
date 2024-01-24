import sys
import time
import glob
import threading
from osp import OSP
import rospy
from std_msgs.msg import Bool

# NOTE connection is boolean variable so far, maybe it will be object in the future
connection = None
PORTS = []
DEVICES = []

def usb_connection(ports):
    global connection, DEVICES

    try:
        devs = list(map(lambda port: OSP(port), ports))
        
        time.sleep(2) # Timeout to receive the first messages from the devices

        print('----------------------------')
        print('devs:', devs)
        print('ports:', ports)
        print('----------------------------')

        for dev in devs:
            dev.osp_info_dev_type()
            time.sleep(1)

        osp_dev_type = 1
        DEVICES = list(filter(lambda d: d.get_dev_type() == osp_dev_type, devs))
        devs_of_wrong_type = filter(lambda d: d.get_dev_type() != osp_dev_type, devs)

        connection = True if bool(DEVICES) else False

        for dev in devs_of_wrong_type:
            dev.stop()

        print('Active threads:', threading.active_count())
    except Exception as err:
        connection = False

def get_os_ports():
    global PORTS
    ports = None
    if sys.platform.startswith('win'):
        ports = ['COM%s' % (i + 1) for i in range(256)]
    elif sys.platform.startswith('linux') or sys.platform.startswith('cygwin'):
        # this excludes your current terminal "/dev/tty"
        ports = glob.glob('/dev/ttyUSB*')
    elif sys.platform.startswith('darwin'):
        ports = glob.glob('/dev/ttyUSB*')
    else:
        raise EnvironmentError('Unsupported platform')

    if PORTS != ports:
        stop_devices()
        usb_connection(ports)
        PORTS = ports

def stop_devices():
    global DEVICES
    for dev in DEVICES:
        dev.stop()


if __name__ == '__main__':
    try:
        rospy.init_node('usb_controller')
        usb_publisher = rospy.Publisher('/usb_connection', Bool, queue_size=10)
        
        rate = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown():
            get_os_ports()
            usb_publisher.publish(connection)
            rate.sleep()

    except Exception as err:
        print(f"Unexpected {err=}, {type(err)=}")
