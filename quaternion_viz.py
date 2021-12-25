import serial, math
def euler_from_quaternion(x, y, z, w):
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)
    return roll_x, pitch_y, yaw_z  # in radians

with serial.Serial('COM8', 115200, timeout=1) as ser:
    while True:
        line = ser.readline()
        # print(line.decode('utf-8'))
        string = line.decode('utf-8')
        if 'q' in string:
            string = string[string.find('q')+2:-1]
            q = string.split('\t')
            r, p, y = euler_from_quaternion(float(q[0]), float(q[1]), float(q[2]), float(q[3]))
            print(r, p, y)