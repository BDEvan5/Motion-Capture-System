import numpy as np
from matplotlib import pyplot as plt


class Data:
    def __init__(self, name="Name Not Set"):
        self.X = []
        self.Y = []
        self.Z = []
        self.d = []
        self.t = []

        self.name = name
        self.stream_data_path = ""

    def add_entry(self, x, y, z, d, t):
        self.X.append(x)
        self.Y.append(y)
        self.Z.append(z)
        self.d.append(d)
        self.t.append(t)

    def plot_data(self):
        # self.condition_data_set(5000)
        fig, axes = plt.subplots(4, 1)

        plt.title(self.name)
        axes[0].plot(self.t, self.X)
        axes[0].set_ylabel('X distance')
        axes[1].plot(self.t, self.Y)
        axes[1].set_ylabel('Y distance')
        axes[2].plot(self.t, self.Z)
        axes[2].set_ylabel('Z distance')
        axes[3].plot(self.t, self.d)
        axes[3].set_ylabel('Total Distance')
        fig.tight_layout()

        # plt.legend()
        # print(self.X)


        plt.show()

    def save_data_collected(self):
        path = self.stream_data_path
        np.save(path + 'X', self.X)
        np.save(path + 'Y', self.Y)
        np.save(path + 'Z', self.Z)
        np.save(path + 't', self.t)
        np.save(path + 'd', self.d)

    def open_data(self, path):
        self.X = np.load(path + 'X.npy', allow_pickle=True)
        self.Y = np.load(path + 'Y.npy', allow_pickle=True)
        self.Z = np.load(path + 'Z.npy', allow_pickle=True)
        self.d = np.load(path + 'd.npy', allow_pickle=True)
        self.t = np.load(path + 't.npy', allow_pickle=True)

    def condition_data(self, list_in, limit):
        new_list = []
        for e in list_in:
            if np.abs(e) > limit:
                # e = limit
                e = 0
            new_list.append(e)
        return new_list

    def condition_data_set(self, lim=2000):
        self.X = self.condition_data(self.X, lim)
        self.Y = self.condition_data(self.Y, lim)
        self.Z = self.condition_data(self.Z, lim)
        self.d = self.condition_data(self.d, lim)

    def run_data_cleanup(self, cleanup_threshold):
        # if the value is more than the threshold away from the previous value, then it takes the average
        for i in range(1, len(self.d)-1):
            if np.abs(self.d[i] - self.d[i-1]) > cleanup_threshold:
                new_val_d = (self.d[i-1] + self.d[i+1]) * 0.5
                self.d[i] = new_val_d
                new_val_X = (self.X[i - 1] + self.X[i + 1]) * 0.5
                self.X[i] = new_val_X
                new_val_Y = (self.Y[i - 1] + self.Y[i + 1]) * 0.5
                self.Y[i] = new_val_Y
                new_val_Z = (self.Z[i - 1] + self.Z[i + 1]) * 0.5
                self.Z[i] = new_val_Z
                print(i)

    def localise_data(self, local=[0, 0, 0]):
        for i in self.t:
            self.X[i] = self.X[i] - local[0]
            self.Y[i] = self.Y[i] - local[1]
            self.Z[i] = self.Z[i] - local[2]
            self.d[i] = np.linalg.norm((self.X[i], self.Y[i], self.Z[i]))

    def estimate_zeros(self):
        zeros = 0
        for i in range(1, len(self.d)-1):
            if self.d[i] == 0:
                zeros += 1
                self.d[i] = (self.d[i+1] + self.d[i-1])/2

        print("There are %d zeros in the data" % zeros)

    def add_vector_entry(self, counter, pos_vec=np.array([0, 0, 0])):
        x = float(pos_vec[0])
        y = float(pos_vec[1])
        z = float(pos_vec[2])
        # print(pos_vec)
        d = (x ** 2 + y ** 2 + z ** 2) ** 0.5
        # print(self.X)
        self.add_entry(x, y, z, d, counter)
