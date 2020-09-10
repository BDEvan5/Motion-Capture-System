import StaticServer
import Calibration
import Analysis
import Triangulation
import ErrorPlot


if __name__ == "__main__":
    Session_number = 7
    Stream_number = 4
    Calibration_number = 4
    size = (640, 480)

    # Sets up server to take calibration run and then record video
    # myServer = StaticServer.Server(Session_number, Stream_number, Calibration_number)
    # myServer.set_up_connection(2)
    # myServer.take_calibration_imgs(20, 2)
    # myServer.take_video(20)
    #
    # myCalib = Calibration.RunCalibration()
    # myCalib.run_calibration(Session_number, Calibration_number, size)
    #
    # myTri = Triangulation.RunTriangulation()
    # myTri.set_up_folder(Session_number, Stream_number, Calibration_number, size)
    # myTri.run_through_videos() # runs through videos with static cameras
    # myTri.run_through_videos_dynamic()  # runs through videos with moving cameras
    # myTri.run_through_videos_marker_size()

    # myTriAnalysis = Analysis.Analysis()
    # myTriAnalysis.set_up_folder(Session_number, Stream_number)
    # myTriAnalysis.run_dynamic_analysis()
    # myTriAnalysis.run_analysis()
    # myTriAnalysis.data.plot_data()
    # myTriAnalysis.plot_results()
    # myTriAnalysis.plot_target()
    # myTriAnalysis.plot_histogram()
    # myTriAnalysis.plot_output()

    # for i in range(6):
    #     Stream_number = i + 1
    #     myTri = Triangulation.RunTriangulation()
    #     myTri.set_up_folder(Session_number, Stream_number, Calibration_number, size)
    #     myTri.run_through_videos()

    # myAnaD = ErrorPlot.AnalysisData()
    # myAnaD.set_path(Session_number, Stream_number)
    # myAnaD.plot_histogram()

    for i in [3, 4, 5]:
        myCalib = Calibration.CameraCalibrationData()
        session_path = 'StaticServerSession%d/' % 7
        calibration_data_path = session_path + "staticCalibrationData%d/" % i
        myCalib.set_parameters(calibration_data_path)
        print(myCalib.projRgt)






