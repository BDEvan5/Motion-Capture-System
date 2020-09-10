import ErrorPlot

if __name__ == "__main__":
    # this file was set up for the experimental plots generated

    # name:
    # indep_name = "Size of Marker (cm)"
    # session = 11
    # streams = [3, 1, 6]
    # indep = [5, 10, 25]
    # streams = [1, 3, 4, 5, 6, 7]
    # indep = [10, 5, 5, 5, 25, 25]

    # name: Resolution
    indep_name = "Resolution X direction (pixels)"
    session = 7
    streams = [3, 4, 5]
    indep = [620, 960, 1280]

    # name:all span
    # indep_name = "Span Between Cameras (cm)"
    # session = 14
    # streams = [10.0, 12, 13, 14, 2, 1, 4, 6, 7, 8]
    # indep = [4, 6, 8, 10, 10, 20, 50, 75, 100, 120]

    # name:small span
    # indep_name = "Span Between Cameras (cm)"
    # session = 9
    # streams = [2, 4, 5, 6]
    # indep = [4, 6, 8, 10]

    # name: Angle
    # indep_name = "Angle Between Camera Setup and Pendulum"
    # session = 8
    # streams = [1, 3, 5, 6, 8, 9]
    # indep = [0, 30, 45, 60, 70, 90]

    # name: repeatability
    # indep_name = "Experimental Run"
    # session = 6
    # streams = [1, 3, 4, 5, 6]
    # indep = [1, 2, 3, 4, 5]

    # # name: distance
    # indep_name = "Distance to marker (cm)"
    # session =  3
    # streams = [2, 3, 4, 5, 6, 7]
    # indep = [1, 1.5, 2, 2.5, 3, 3.5]
    #
    # # name: span
    # indep_name = "Span Between Cameras (cm)"
    # session = 4
    # streams = [2, 1, 4, 6, 7, 8]
    # indep = [10, 20, 50, 75, 100, 120]

    myErr = ErrorPlot.ErrorPlot(session, streams, indep)
    myErr.calculate_errors()
    myErr.plot_graph(indep_name)
    # myErr.plot_errors()

    # # name:
    # indep_name = ""
    # session =
    # streams = []
    # indep = []
