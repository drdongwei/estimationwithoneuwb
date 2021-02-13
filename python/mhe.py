import pickle
import numpy as np
from scipy.optimize import *
import matplotlib.pyplot as plt
import tf
import scipy.io as scio

class MHE:
    def __init__(self, tag=2):
        f = open(str(tag)+"c.pkl", 'rb')
        self.data = pickle.load(f)
        f.close()
        self.x = []
        self.y = []
        self.u = []
        self.attref = []
        self.dt = []
        self.tag = tag

    def timeAlign(self):
        mat = {}
        t_uwb = self.data["uwbuav"][3, :] - self.data["uwbuav"][3, 0]
        t_imu = self.data["IMU"][0, :] - self.data["IMU"][0, 0]
        t_attref = self.data["attref"][0, :] - self.data["attref"][0, 0]
        t_attitude = self.data["attitude"][0, :] - self.data["attitude"][0, 0]
        tgtdata = self.data["gtdata"]
        self.gtdata = np.zeros((len(t_uwb), 3))
        self.gtvel = np.zeros((len(t_uwb), 3))
        t_gt = tgtdata[0, :] - tgtdata[0, 0]
        self.time = t_uwb
        self.x = np.zeros((len(t_uwb), 6))
        self.y = self.data["uwbuav"][1, :]
        self.y1 = self.data["uwbuav"][4, :]
        self.y2 = self.data["uwbuav"][5, :]
        self.u = np.zeros((len(t_uwb), 3))
        self.attref = np.zeros((len(t_uwb), 3))
        self.attitude = np.zeros((len(t_uwb), 3))
        self.dt = np.zeros(len(t_uwb))
        # estimate the zero drift of the imu
        da = [0, 0, 0]
        da[0] = np.average(self.data["IMU"][1, :])
        da[1] = np.average(self.data["IMU"][2, :])+0.06
        da[2] = np.average(self.data["IMU"][3, :])
        att_index = 1 # index of attitude reference
        ratt_index = 0 # index of attitude time stamp
        for i in range(len(t_uwb)-1):
            if (t_uwb[i] > t_attref[att_index]) and (att_index < len(t_attref)-1):
                att_index = att_index + 1
            r = (t_uwb[i] - t_attref[att_index - 1])/(t_attref[att_index] - t_attref[att_index - 1])
            if t_uwb[i] > t_attitude[ratt_index]:
                ratt_index = ratt_index + 1

            for j in range(3):
                timu = self.data["IMU"][j+1, ((t_imu > t_uwb[i]) & (t_imu < t_uwb[i+1]))]
                self.u[i, j] = np.average(timu) #- da[j]
                jgt = tgtdata[j+1, ((t_gt > t_uwb[i]) & (t_gt < t_uwb[i+1]))]
                self.gtdata[i, j] = np.average(jgt)
                if j == 1:
                    self.u[i, j] = self.u[i, j]
            self.attref[i, 0] = self.data["attref"][1, att_index-1] + r*(self.data["attref"][1, att_index] - self.data["attref"][1, att_index-1])
            self.attref[i, 1] = self.data["attref"][2, att_index-1] + r*(self.data["attref"][2, att_index] - self.data["attref"][2, att_index-1])
            self.attref[i, 2] = self.u[i, 2]
            #self.attref[i, 0] = 9.8*self.attref[i, 0]
            #self.attref[i, 1] = -9.8*self.attref[i, 1]
            R = tf.transformations.euler_matrix(self.data["attitude"][1, ratt_index], self.data["attitude"][2, ratt_index], self.data["attitude"][3, ratt_index], 'sxyz')
            R = R[0:3, 0:3]
            self.u[i, :] = (R*np.asmatrix(self.u[i, :]).T).T
            self.attitude[i, 0] = self.data["attitude"][1, ratt_index]
            self.attitude[i, 1] = self.data["attitude"][2, ratt_index]
            self.attitude[i, 2] = self.data["attitude"][3, ratt_index]
            self.dt[i] = t_uwb[i+1] - t_uwb[i]
            self.gtdata[i, :] = self.gtdata[i, :] - [-4.9086, -0.0136, 0.5221]
            if i>0:
                self.gtvel[i-1, :] = (self.gtdata[i, :] - self.gtdata[i-1, :])/self.dt[i-1]
        self.gtvel[-1, :] = self.gtvel[-2, :]
        mat["time"] = t_uwb
        mat["uwb"] = self.y
        mat["uwb1"] = self.y1
        mat["uwb2"] = self.y2
        mat["imu"] = self.u
        mat["gtd"] = self.gtdata
        mat["gtv"] = self.gtvel
        mat["att"] = self.attitude
        scio.savemat(str(self.tag)+".mat", mat)
        self.u[:, 0] = self.u[:, 0] - np.average(self.u[:, 0])
        self.u[:, 1] = self.u[:, 1] - np.average(self.u[:, 1])
        self.u[:, 2] = self.u[:, 2] - np.average(self.u[:, 2])
        self.u[:, 1] = -self.u[:, 1]
        print([np.average(self.u[:, 0]), np.average(self.u[:, 1]), np.average(self.u[:, 2])])

    def objective(self, x, y, u, dt):
        obj = 0
        xt = x
        for i in range(len(y)-1):
            if i > 0:
                for j in range(3):
                    xt[j] = xt[j] + xt[j+3] * dt[i] + 0.5 * u[i-1, j] * dt[i] * dt[i]
                for j in range(3):
                    xt[j+3] = xt[j+3] + u[i-1, j] * dt[i]
            tobj = xt[0]*xt[0] + xt[1] * xt[1] + xt[2]*xt[2] -y[i] * y[i]
            obj = obj + tobj * obj
        return obj

    def propagation(self, x, u, dt, r = [0, -1]):
        if(r[1]<0):
            x = x[r[0]:, :]
            u = u[r[0]:, :]
            dt = dt[r[0]:]
        else:
            x = x[r[0]:r[1], :]
            u = u[r[0]:r[1], :]
            dt = dt[r[0]:r[1]]
        for i in range(len(dt)):
            for j in range(3):
                x[i, j] = x[i-1, j] + x[i-1, j+3] * dt[i] + 0.5 * u[i-1, j] * dt[i] * dt[i]
                x[i, j+3] = x[i-1, j+3] + u[i-1, j] * dt[i]
        return x

    #def propagation_with_observer(self, x, u, dt):

    def att2acc(self, att):
        u = np.zeros((len(self.time), 3))
        u[:, 0] = att[:, 1] * 9.8
        u[:, 1] = -att[:, 0] * 9.8
        u[:, 2] = self.u[:, 2]
        return u

    def prop_imu(self):
        x = np.zeros((len(self.time), 6))
        x = self.propagation(x, self.u, self.dt)
        return x

    def prop_attref(self):
        x = np.zeros((len(self.time), 6))
        u = self.att2acc(self.attref)
        x = self.propagation(x, u, self.dt)
        return x

    def prop_att(self, att):
        x = np.zeros((len(self.time), 6))
        u = self.att2acc(att)
        x = self.propagation(x, u, self.dt)
        return x

    def euclidean(self, x):
        d = np.zeros(len(x[:, 0]))
        for i in range(len(x[:, 0])):
            d[i] = np.linalg.norm(x[i, :])
        return d

    def evalimudrift(self):
        x = self.prop_imu()
        dx_im = x[-1, 3:] - x[0, 3:]
        dx_gt = self.gtvel[-2, :] - self.gtvel[0, :]
        t_total = self.time[-1]
        dx = dx_im - dx_gt
        dimu = dx / t_total
        return dimu


    def eval(self):
        x0 = np.zeros(6)
        r = 10
        bds = [(-r, r), (-r, r), (-r, r), (-r, r), (-r, r), (-r, r)]
        res = minimize(self.objective, x0, bounds=bds, args=(self.y, self.u, self.dt), method='Nelder-Mead', options={'xatol': 1e-3, 'disp': True})
        #res = shgo(self.objective, bounds, args=(self.y, self.u, self.dt))
        print(res.x)
        u = self.u
        u[:, 0] = self.attref[:, 0]
        u[:, 1] = self.attref[:, 1]
        dt = self.dt
        for i in range(1, len(self.dt)):
            for j in range(3):
                self.x[i, j] = self.x[i-1, j] + self.x[i-1, j+3] * dt[i] + 0.5 * u[i-1, j] * dt[i] * dt[i]
            for j in range(3):
                self.x[i, j+3] = self.x[i-1, j+3] + u[i-1, j] * dt[i]

    def compare_different(self):
        x0 = np.zeros((len(t_uwb), 3))



if __name__== "__main__":
    mhe = MHE()
    mhe.timeAlign()
    exit()
    #dgt = mhe.gtdata[1:3]
    dimu = mhe.evalimudrift()
    print("The estimated imu drift is: ")
    print(dimu)
    x_imu = mhe.prop_imu()
    d_imu = mhe.euclidean(x_imu)
    #print(mhe.attitude)
    x_att = mhe.prop_att(mhe.attitude)
    d_att = mhe.euclidean(x_att)
    x_ref = mhe.prop_att(mhe.attref)
    d_ref = mhe.euclidean(x_ref)
    gtd = np.zeros((len(mhe.time), 3))
    #for i in range(len(mhe.gtdata[:, 0])):
    #    gtd[i, :] = mhe.gtdata[i, :] - mhe.gtdata[0, :]
    d_gtd = mhe.euclidean(mhe.gtdata)
    #print(d_gtd)
    plt.figure()
    plt.plot(mhe.time, mhe.y, 'r')
    plt.plot(mhe.time, d_imu, 'g')
    #plt.plot(mhe.time, d_att, 'b')
    #plt.plot(mhe.time, d_ref, 'y')
    plt.plot(mhe.time, d_gtd, 'k')
    plt.grid()
    plt.legend(['UWB', 'IMU', r'$\theta$', r'$\theta^*$', 'GT'])
    plt.figure()
    plt.subplot(311)
    plt.plot(mhe.time, mhe.gtvel[:, 0], 'r')
    plt.plot(mhe.time, mhe.gtvel[:, 1], 'g')
    plt.plot(mhe.time, mhe.gtvel[:, 2], 'b')
    plt.grid()
    plt.subplot(312)
    plt.plot(mhe.time, x_imu[:, 3], 'r')
    plt.plot(mhe.time, x_imu[:, 4], 'g')
    plt.plot(mhe.time, x_imu[:, 5], 'b')
    plt.grid()
    plt.subplot(313)
    plt.plot(mhe.time, x_att[:, 3], 'r')
    plt.plot(mhe.time, x_att[:, 4], 'g')
    plt.plot(mhe.time, x_att[:, 5], 'b')
    plt.grid()
    plt.show()
    exit()
    plt.figure()
    u = mhe.att2acc(mhe.attitude)
    plt.plot(mhe.time, u[:, 2],'r')
    plt.plot(mhe.time, mhe.u[:, 2]+0.1,'g')
    plt.show()
    mhe.eval()
    plt.figure(1)
    plt.subplot(311)
    plt.plot(mhe.time, mhe.x[:, 0], 'r')
    plt.plot(mhe.time, mhe.x[:, 1], 'g')
    plt.plot(mhe.time, mhe.x[:, 2], 'b')
    plt.subplot(312)
    plt.plot(mhe.data["gtdata"][0, :], mhe.data["gtdata"][1, :], 'r')
    plt.plot(mhe.data["gtdata"][0, :], mhe.data["gtdata"][2, :], 'g')
    plt.plot(mhe.data["gtdata"][0, :], mhe.data["gtdata"][3, :], 'b')
    plt.subplot(313)
    plt.plot(mhe.time, mhe.u[:, 0], 'r')
    plt.plot(mhe.time, mhe.u[:, 1], 'g')
    plt.plot(mhe.time, mhe.u[:, 2], 'b')
    plt.figure(2)
    plt.subplot(311)
    plt.plot(mhe.time, mhe.x[:, 3], 'r')
    plt.plot(mhe.time, mhe.x[:, 4], 'g')
    plt.plot(mhe.time, mhe.x[:, 5], 'b')
    plt.grid()
    plt.subplot(312)
    xv = np.zeros((len(mhe.data["gtdata"][0, :]), 3))
    for i in range(1, len(mhe.data["gtdata"][0, :])):
        dt = mhe.data["gtdata"][0, i] - mhe.data["gtdata"][0, i-1]
        xv [i, :] = [(mhe.data["gtdata"][1, i] - mhe.data["gtdata"][1, i-1])/dt, (mhe.data["gtdata"][2, i] - mhe.data["gtdata"][2, i-1])/dt, (mhe.data["gtdata"][3, i] - mhe.data["gtdata"][3, i-1])/dt]

    plt.plot(mhe.data["gtdata"][0, :], xv [:, 0], 'r')
    plt.plot(mhe.data["gtdata"][0, :], xv [:, 1], 'g')
    plt.plot(mhe.data["gtdata"][0, :], xv [:, 2], 'b')
    plt.subplot(313)
    plt.plot(mhe.time, mhe.attitude[:,0])
    plt.plot(mhe.time, mhe.attitude[:,1])
    plt.grid()
    # plt.figure(3)
    # plt.plot(mhe.data["IMU"][1, :])
    plt.show()
