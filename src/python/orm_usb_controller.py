import sys
import glob
from osp import OSP

def findOutOSAndGetPorts():
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
    return ports


if __name__ == '__main__':
    try:
        ports = findOutOSAndGetPorts()
        for port in ports:
            orm = OSP(port)
            print(orm.joint_angle)
    except Exception as err:
        print(f"Unexpected {err=}, {type(err)=}")
        print("Connection lost...")
