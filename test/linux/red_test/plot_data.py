import matplotlib.pyplot as plt

with open('/home/devel/projects/opus/opus/ros_ws/src/opus_ethercat_sdk/soem_interface/soem_ros2/SOEM/build/test/linux/red_test/out_curr.txt') as f:
    lines = f.readlines()
    slaves = [int(line.split()[0]) for line in lines]
    x_or = [int(line.split()[1]) for line in lines]
    y_curr_or = [int(line.split()[2]) for line in lines]
    z_curr_or = [int(line.split()[3]) for line in lines]
    w_curr_or = [int(line.split()[4]) for line in lines]

with open('/home/devel/projects/opus/opus/ros_ws/src/opus_ethercat_sdk/soem_interface/soem_ros2/SOEM/build/test/linux/red_test/out_med.txt') as f:
    lines = f.readlines()
    slaves = [int(line.split()[0]) for line in lines]
    x_or = [int(line.split()[1]) for line in lines]
    y_med_or = [int(line.split()[2]) for line in lines]
    z_med_or = [int(line.split()[3]) for line in lines]
    w_med_or = [int(line.split()[4]) for line in lines]

    minim = min(x_or)
    x_or = [elem - minim for elem in x_or]

slaves_set = set(slaves)
indexes = []
for slave in slaves_set:
    indexes.append([index for index, val in enumerate(zip(x_or,slaves)) if val[1] == slave])

for slave in slaves_set:
    slave = slave

    x = [val for val, slave_val in zip(x_or,slaves) if slave_val == slave]
    y_curr = [val for val, slave_val in zip(y_curr_or,slaves) if slave_val == slave]
    y_med = [val for val, slave_val in zip(y_med_or,slaves) if slave_val == slave]
    z_curr = [val for val, slave_val in zip(z_curr_or,slaves) if slave_val == slave]
    z_med = [val for val, slave_val in zip(z_med_or,slaves) if slave_val == slave]
    w_curr = [val for val, slave_val in zip(w_curr_or,slaves) if slave_val == slave]
    w_med = [val for val, slave_val in zip(w_med_or,slaves) if slave_val == slave]

    plt.figure(slave)
    plt.title("Slave {}".format(slave))

    plt.subplot(231)
    plt.plot(x, y_curr, 'b.', x, y_med, 'k')
    plt.ylabel('Torque [mNm]', fontsize = 10) #for y label
    plt.xlabel('Time [tick]', fontsize = 10) #for x label

    plt.subplot(234)
    plt.plot(x, y_med, 'r--')
    plt.ylabel('Med Torque [mNm]', fontsize = 10) #for y label
    plt.xlabel('Time [tick]', fontsize = 10) #for x label

    plt.subplot(232)
    plt.plot(x, z_curr, 'b.', x, z_med, 'k')
    plt.ylabel('Current [mA]', fontsize = 10) #for y label
    plt.xlabel('Time [tick]', fontsize = 10) #for x label

    plt.subplot(235)
    plt.plot(x, z_med, 'r--')
    plt.ylabel('Med Current [mA]', fontsize = 10) #for y label
    plt.xlabel('Time [tick]', fontsize = 10) #for x label

    plt.subplot(233)
    plt.plot(x, w_curr, 'b.', x, w_med, 'k')
    plt.ylabel('Velocity [pulse/sec]', fontsize = 10) #for y label
    plt.xlabel('Time [tick]', fontsize = 10) #for x label

    plt.subplot(236)
    plt.plot(x, w_med, 'r--')
    plt.ylabel('Med Velocity [pulse/sec]', fontsize = 10) #for y label
    plt.xlabel('Time [tick]', fontsize = 10) #for x label

    plt.xlim([min(x), max(x)])

    plt.tight_layout()

plt.show()
