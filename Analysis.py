import cv2 as cv
import numpy as np
import Triangulation as T
import Data as D
from matplotlib import pyplot as plt
import os


class Analysis:
    def __init__(self):
        self.type = "Static"
        self.session = ""
        self.stream = ""
        # self.calib = ""

        self.session_dp = ""
        self.stream_dp = ""
        self.analysis_dp = ""
        # self.calib_dp = ""

        self.data = D.Data()
        self.target = D.Data()
        self.error = D.Data()
        self.x = []
        self.y = []

    def set_up_folder(self, session, stream):
        self.session = session
        self.stream = stream

        self.session_dp = self.type + "ServerSession%d/" % self.session
        self.stream_dp = self.session_dp + "StreamData%d/" % self.stream
        self.analysis_dp = self.stream_dp + "Analysis%d/" % self.stream

        if not os.path.exists(self.analysis_dp):
            os.mkdir(self.analysis_dp)

        self.data.stream_data_path = self.stream_dp
        self.data.open_data(self.stream_dp)
        # self.data.plot_data()

    def run_analysis(self):
        condition_val = 100
        trim_val = 49
        frame_rate = 10
        # self.data.plot_data()
        # self.data.estimate_zeros()
        # self.relocalise_data()
        self.data.condition_data_set(condition_val)
        # self.data.run_data_cleanup(12)
        self.trim_data(trim_val)
        self.set_model_target(72, frame_rate)
        self.calculate_error()
        self.run_stats_analysis(10)
        self.save_analysis_data()
        # self.plot_results()

    def run_dynamic_analysis(self):
        condition_val = 100
        self.data.condition_data_set(condition_val)
        self.error.d = np.zeros((0, len(self.data.t)))

    def plot_results(self):
        fig, axes = plt.subplots(4, 1)
        axes[0].plot(self.data.t, self.data.d)
        axes[0].set_ylabel('Triangulated')
        axes[1].plot(self.target.X, self.target.Y)
        axes[1].set_ylabel('Target')
        axes[2].plot(self.error.t, self.error.d)
        axes[2].set_ylabel('Error')
        axes[2].set_ylim(0, 10)
        axes[3].plot(self.data.t, self.data.d)
        axes[3].plot(self.target.X, self.target.Y)
        axes[3].set_ylabel('Combined')
        fig.tight_layout()
        plt.legend()
        plt.savefig(self.analysis_dp + 'AnalysisPlot2.png')
        plt.show()

    def plot_output(self):
        fig = plt.figure(1)
        plt.plot(self.data.t, self.data.d, 'x', label='Triangulated displacement', markersize=12, color='black')
        plt.plot(self.target.X, self.target.Y, label='True displacement')
        plt.ylabel('Displacement (cm)')
        plt.xlabel('Time (frames)')
        plt.legend()
        plt.show()

    def set_model_target(self, amplitude, framerate=10):
        self.target.t = self.data.t
        self.target.X = np.arange(0, len(self.data.t), 0.1)
        zeta = 0.11 / framerate
        # zeta = 0.06 / framerate
        # omega = 29.75 * 0.5 / (2 * np.pi * framerate)
        omega = 30 * 0.5 / (2 * np.pi * framerate)
        self.target.d = amplitude * np.sin(self.target.t * omega) * np.exp(-zeta * self.target.t)
        self.target.Y = amplitude * np.sin(self.target.X * omega) * np.exp(-zeta * self.target.X)
        self.target.d = np.abs(self.target.d)
        self.target.Y = np.abs(self.target.Y)

        self.x = self.target.d * np.cos(self.target.t * omega/2)
        self.y = self.target.d * np.sin(self.target.t * omega/2)

    def plot_target(self):
        error_x = []
        error_y = []
        error_d = []
        xy = zip(self.x, self.y)
        i = 0
        for x, y in xy:
            error_x.append(self.data.X[i] - x)
            error_y.append(self.data.Y[i] - y)
            error_d.append((error_x[i]**2 + error_y[i]**2)**0.5)
            i += 1

        avg_error = np.average(error_d)
        std_dev = np.std(error_d)
        print(avg_error, std_dev)
        avg_error ,std_dev = self.run_stats_analysis(10)

        plt.plot(error_x, error_y, 'x', label='Error Points')
        plt.axvline(x=0, color='black')
        plt.axhline(y=0, color='black')
        aveCirc = plt.Circle((0, 0), avg_error, fill=False, label='Average Error', color='red')
        stdCirc = plt.Circle((0, 0), (avg_error + std_dev), fill=False, label='1 Standard Deviation', color='green')
        ax = plt.gca()
        ax.add_artist(aveCirc)
        ax.add_artist(stdCirc)
        plt.ylim((-10, 10))
        plt.xlim((-10, 10))
        plt.ylabel('Error in Y Direction (cm)')
        plt.xlabel("Error in X Direction (cm)")
        plt.title("Error Location")
        plt.legend()
        plt.show()

    def trim_data(self, trim_val):
        if trim_val != 0:
            i = np.arange(trim_val - 1)
            j = np.ones(trim_val - 1) * (len(self.data.t) - 1) - i
            self.data.d = np.delete(self.data.d, i)
            self.data.t = np.delete(self.data.t, j)

    def calculate_error(self):
        zero_data_counter = 0
        for i in range(len(self.data.d)):
            if self.data.d[i] != 0:
                self.error.d.append(self.target.d[i] - self.data.d[i])
            else:
                self.error.d.append(0)
                zero_data_counter += 1

        print(zero_data_counter)
        self.error.t = self.data.t
        self.error.d = np.abs(self.error.d)

    def run_stats_analysis(self, start_point=10):
        error = self.error.d[start_point:]
        avg_error = np.average(error)
        std_dev = np.std(error)

        print("Average error: %f" % avg_error)
        print("Standard Deviation: %f" % std_dev)

        # translation = np.load(self.calib_data_path + 'translation.npy')
        # print(translation)
        # print(np.linalg.norm(translation))

        return avg_error, std_dev

    def save_analysis_data(self):
        path = self.analysis_dp
        np.save(path + 'data_d', self.data.d)
        np.save(path + 'data_t', self.data.t)
        np.save(path + 'target_t', self.target.X)
        np.save(path + 'target_d', self.target.Y)
        np.save(path + 'error_e', self.error.d)

    def relocalise_data(self):
        temp = self.data.d[39:]
        avg = np.array(temp).mean()
        print(avg)
        self.data.d -= np.ones((len(self.data.d))) * avg
        self.data.d = np.abs(self.data.d)

    def plot_histogram(self):
        # possibly remove zeros first
        error = np.copy(self.error.d)
        error = [e for e in error if (e != 0 and e < 20)]
        print(error)

        n, bins, patches = plt.hist(x=error, bins='auto', color='#0504aa')
                                    # alpha=0.7, rwidth=0.85)
        plt.grid(axis='y', alpha=0.75)
        plt.xlabel('Error (cm)')
        plt.ylabel('Frequency')
        plt.title('Distribution of Error')
        # plt.text(23, 45, r'$\mu=15, b=3$')
        maxfreq = n.max()
        # Set a clean upper y-axis limit.
        plt.ylim(ymax=np.ceil(maxfreq / 10) * 10 if maxfreq % 10 else maxfreq + 10)
        plt.show()


