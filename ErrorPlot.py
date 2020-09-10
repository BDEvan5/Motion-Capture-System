
from matplotlib import pyplot as plt
import scipy.io as sio
import numpy as np


class ErrorPlot:
    def __init__(self, session=1, stream_nums=[1, 2, 3], indep=[1, 2, 3]):
        self.session = session
        self.stream = stream_nums
        self.ses_length = len(self.stream)
        self.analysis_list = []
        self.indep = indep
        for i in range(self.ses_length):
            obj = AnalysisData()
            obj.set_path(self.session, self.stream[i])
            self.analysis_list.append(obj)

        self.error_list = []
        self.std_list = []

    def calculate_errors(self):
        for obj, i in zip(self.analysis_list, range(len(self.analysis_list))):
            print("Stream: " + str(self.stream[i]) + " --> Independent variable: " + str(self.indep[i]))
            err, std = obj.run_error_analysis()
            self.error_list.append(err)
            self.std_list.append(err)

    def plot_graph(self, indep_name):
        plt.figure(1)
        plt.plot(self.indep, self.error_list, 'x', markersize=12, color='red')
        # plt.plot(self.indep, self.std_list)
        plt.ylabel('Average Error (cm)')

        # self.error_list[:] = self.error_list[:,0]
        diction = {'x':self.indep, 'y':self.error_list}
        sio.savemat('Experimental.mat', diction)
        plt.xlabel(indep_name)
        plt.title('Average Error vs ' + indep_name)
        plt.show()

    def plot_errors(self):
        plt.figure(2)
        fig, axes = plt.subplots(self.ses_length+1, 1)
        for (obj, i) in zip(self.analysis_list, range(self.ses_length)):
            axes[i].plot(obj.t, obj.error)
        plt.title('Error in experiments')

        for obj in self.analysis_list:
            axes[self.ses_length].plot(obj.t, obj.error)
        plt.show()


class AnalysisData:
    def __init__(self):
        self.d = []
        self.t = []
        self.targetX = []
        self.targetY = []
        self.error = []

        self.path = ""

    def set_path(self, session, stream):
        session_dp = "StaticServerSession%d/" % session
        stream_dp = session_dp + "StreamData%d/" % stream
        self.path = stream_dp + "Analysis%d/" % stream

        self.load_analysis_data()

    def load_analysis_data(self):
        path = self.path
        self.d = np.load(path + 'data_d.npy', allow_pickle=True)
        self.t = np.load(path + 'data_t.npy', allow_pickle=True)
        self.targetX = np.load(path + 'target_t.npy', allow_pickle=True)
        try:
            self.targetY = np.load(path + 'target d.npy', allow_pickle=True)
        except:
            self.targetY = np.load(path + 'target_d.npy', allow_pickle=True)
        self.error = np.load(path + 'error_e.npy', allow_pickle=True)

    def run_error_analysis(self):
        start_point = 10
        error = self.error[start_point:]
        # print(error)
        avg_error = np.average(error)
        std_dev = np.std(error)

        print("Average error: %f" % avg_error)
        print("Standard Deviation: %f" % std_dev)
        return avg_error, std_dev

    def plot_histogram(self):
        # avg_error, std_dev = self.run_error_analysis()

        # possibly remove zeros first
        error = np.copy(self.error)
        error = [e for e in error if (e != 0 and e < 10)]
        # print(error)

        avg_error = np.average(error)
        std_dev = np.std(error)
        median = np.median(error)

        n, bins, patches = plt.hist(x=error, bins=30, color='#0504aa', alpha=0.7, rwidth=0.85, label='Frequency of Error')

        plt.axvline(x=avg_error, color='red', label='Average Error')
        plt.axvline(x=(avg_error+std_dev), linestyle='--', color='green', label='1 Standard Deviation')
        plt.axvline(x=(avg_error - std_dev), linestyle='--', color='green')
        plt.axvline(x=median, linestyle=':', color='blue', label='Median')
        plt.grid(axis='y', alpha=0.75)
        plt.xlabel('Error (cm)')
        plt.ylabel('Frequency')
        plt.title('Distribution of Error')
        plt.legend()
        maxfreq = n.max()
        # Set a clean upper y-axis limit.
        plt.ylim(ymax=np.ceil(maxfreq / 10) * 10 if maxfreq % 10 else maxfreq + 10)
        plt.show()



