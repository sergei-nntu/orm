import sys
import time
import glob
import threading
from osp import OSP
import rospy
from std_msgs.msg import Bool


class USBController:

    def __init__(self):
        self.ports = []
        self.devices = []
        self.osp_dev_type = 1        
        self.connection = False

    def process(self):
        self.verify_ports()
        self.logger()
        self.establish_connection()

    def verify_ports(self):
        ports = self.get_os_ports()
        if self.ports_is_not_same(ports):
            self.stop_devices()
            self.connect(ports)
            self.set_ports(ports)

    def connect(self, ports):
        devs = self.get_osp_dev(ports)
        self.get_osp_dev_type(devs)
        self.receive_devs_of_type(devs)
        self.close_devs_of_wrong_type(devs)

    def get_osp_dev(self, ports):
        try:
            devs = list(map(lambda port: OSP(port), ports))
            time.sleep(2) # Timeout to receive the first messages from the devices
            return devs
        except Exception as err:
            print("Error!!!!", err)
            self.connection = False
            return []

    def get_osp_dev_type(self, devs):
        for dev in devs:
            dev.osp_info_dev_type()
            time.sleep(1)

    def receive_devs_of_type(self, devs):
        self.devices = list(filter(lambda d: d.get_dev_type() == self.osp_dev_type, devs))

    def close_devs_of_wrong_type(self, devs):
        devs_of_wrong_type = filter(lambda d: d.get_dev_type() != self.osp_dev_type, devs)
        for dev in devs_of_wrong_type:
            dev.stop()

    def establish_connection(self):
        self.connection = True if bool(self.devices) else False

    def logger(self):
        print('----------------------------')
        print('Devices:', self.devices)
        print('Ports:', self.ports)
        print('Active threads:', threading.active_count())
        print('----------------------------')

    def set_ports(self, ports):
        self.ports = ports

    def ports_is_not_same(self, ports):
        return self.ports != ports

    def stop_devices(self):
        for dev in self.devices:
            dev.stop()
        time.sleep(1) # need to close ports

    def get_os_ports(self):
        ports = None
        if sys.platform.startswith('win'):
            ports = ['COM%s' % (i + 1) for i in range(256)]
        elif sys.platform.startswith('linux') or sys.platform.startswith('cygwin'):
            # this excludes your current terminal "/dev/tty"
            lin_ports = glob.glob('/dev/ttyUSB*')
            mac_ports = glob.glob('/dev/tty.usb*')
            ports = lin_ports if not mac_ports else mac_ports
        elif sys.platform.startswith('darwin'):
            ports = glob.glob('/dev/ttyUSB*')
        else:
            raise EnvironmentError('Unsupported platform')
        return ports


if __name__ == '__main__':
    try:
        rospy.init_node('usb_controller')
        usb_publisher = rospy.Publisher('/usb_connection', Bool, queue_size=10)

        usb_controller = USBController()

        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            usb_controller.process()
            usb_publisher.publish(usb_controller.connection)
            rate.sleep()

    except Exception as err:
        print(f"Unexpected {err=}, {type(err)=}")
