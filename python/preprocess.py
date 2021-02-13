from unpackrosbag import Unpacker
import matplotlib.pyplot as plt
import os
import pickle
import numpy as np

imudt = 0
num=2
data={}
if os.path.exists(str(num)+".pkl"):
    f = open(str(num)+".pkl", 'rb')
    data = pickle.load(f)
    f.close()
else:
    upacker = Unpacker(num)
    upacker.unpack()
    data["uwbuav"] = upacker.fetchUAV_UWB()
    data["uwbugv"] = upacker.fetchUGV_UWB()
    data["IMU"] = upacker.fetchIMU()
    data["gtPos"] = upacker.fetchGroundTruth()
    data["attRef"] = upacker.fetchAttref()
    data["attitude"] = upacker.fetchPose()
    f = open(str(num)+".pkl", 'wb')
    pickle.dump(data, f, protocol=pickle.HIGHEST_PROTOCOL)
    f.close()

uavt0 = data["uwbuav"][3, 0]
ugvt0 = data["uwbugv"][3, 0]
time0 = max(data["uwbuav"][3, 0], data["uwbugv"][3, 0]) + 2
timee = min(data["uwbuav"][3, -1], data["uwbugv"][3, -1]) - 33
t = data["uwbuav"][3, ((data["uwbuav"][3, :] > time0) & (data["uwbuav"][3, :] < timee))]
t = t - t[0]
uavuwb = data["uwbuav"][:, ((data["uwbuav"][3, :] > time0) & (data["uwbuav"][3, :] < timee))]
ugvuwb = data["uwbugv"][:, ((data["uwbugv"][3, :] > time0) & (data["uwbugv"][3, :] < timee))]

imudt0 = time0 - uavt0 + imudt
imudte = timee - uavt0 + imudt
imudata = data["IMU"][:, ((data["IMU"][0, :] > data["IMU"][0, 0] + imudt0) & (data["IMU"][0, :] < data["IMU"][0, 0] + imudte))]
attref = data["attRef"][:, ((data["attRef"][0, :] > data["attRef"][0, 0] + imudt0) & (data["attRef"][0, :] < data["attRef"][0, 0] + imudte))]
attitude = data["attitude"][:, ((data["attitude"][0, :] > data["attitude"][0, 0] + imudt0) & (data["attitude"][0, :] < data["attitude"][0, 0] + imudte))]
gtt0 = time0 - ugvt0
gtte = timee - ugvt0
gtdata = data["gtPos"][:,((data["gtPos"][0,:] > data["gtPos"][0,0] + gtt0) & (data["gtPos"][0, :] < data["gtPos"][0, 0] + gtte))]

dat = {}
dat['IMU'] = imudata
dat['uwbuav'] = uavuwb
dat['uwbugv'] = ugvuwb
dat['gtdata'] = gtdata
dat['attref'] = attref
dat['attitude'] = attitude
f = open(str(num)+"c.pkl", 'wb')
pickle.dump(dat, f, protocol=pickle.HIGHEST_PROTOCOL)
f.close()

exit()

plt.figure(1)
plt.subplot(311)
#print(data["uwbuav"][:, 0])
plt.plot(t, uavuwb[1, :], 'b')
plt.plot(t, ugvuwb[1, :] + 0.2, 'r')
plt.grid()
plt.subplot(312)
flen = 100
plt.plot(imudata[0, :] - imudata[0, 0], np.convolve(imudata[1, :], np.ones((flen,))/flen, mode = 'same'), 'r')
plt.plot(imudata[0, :] - imudata[0, 0], np.convolve(imudata[2, :], np.ones((flen,))/flen, mode = 'same'), 'g')
#plt.plot(imudata[0, :] - imudata[0, 0], np.convolve(imudata[3, :], np.ones((flen,))/flen, mode = 'same')-9.8, 'b')
plt.grid()
plt.subplot(313)
plt.plot(gtdata[0, :] - gtdata[0, 0], gtdata[1, :], 'r')
plt.plot(gtdata[0, :] - gtdata[0, 0], gtdata[2, :], 'g')
plt.plot(gtdata[0, :] - gtdata[0, 0], gtdata[3, :], 'b')
plt.grid()
plt.figure(2)
plt.subplot(311)
plt.plot(attref[0, :] - attref[0, 0], attref[1, :], 'r')
plt.plot(attref[0, :] - attref[0, 0], attref[2, :], 'g')
plt.plot(attref[0, :] - attref[0, 0], attref[3, :], 'b')
plt.grid()
plt.subplot(312)
plt.plot(attitude[0, :] - attitude[0, 0], attitude[1, :], 'r')
plt.plot(attitude[0, :] - attitude[0, 0], attitude[2, :], 'g')
plt.plot(attitude[0, :] - attitude[0, 0], attitude[3, :], 'b')
plt.grid()


plt.show()
