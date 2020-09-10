import Data
import cv2 as cv
import numpy as np
import os
import cv2.aruco as aruco
import glob
import Calibration


class RunTriangulation:
    def __init__(self):
        self.stream_data_path = ""
        self.session_path = ""
        self.calibration_data_path = ""
        self.data = Data.Data()
        self.stream_number = 1
        self.image_size = (0, 0)
        self.ptc = TriangulationCalculation()

        self.marker_location_data = Data.Data("Marker Location")
        self.camera_location_data = Data.Data("Camera Location")

    def set_up_folder(self, session, stream_number, calibration_num, image_size=(640, 480)):
        self.session_num = session
        self.session_path = 'StaticServerSession%d/' % self.session_num
        self.stream_number = stream_number

        self.stream_data_path = self.session_path + 'StreamData%d/' % stream_number
        if not os.path.exists(self.stream_data_path):
            os.mkdir(self.stream_data_path)

        self.calibration_data_path = self.session_path + 'staticCalibrationData%d/' % calibration_num

        self.marker_location_data.stream_data_path = self.stream_data_path

        self.image_size = image_size
        self.ptc.set_parameters(self.calibration_data_path, self.image_size, 12, 4)
        # returns the strem data path so that server saves images in right place
        return self.stream_data_path

    def run_through_videos(self):
        left_name = self.stream_data_path + 'LeftVideo%dClean.avi' % self.stream_number
        right_name = self.stream_data_path + 'RightVideo%dClean.avi' % self.stream_number
        left_video = cv.VideoCapture(left_name)
        right_video = cv.VideoCapture(right_name)

        counter = 0
        while True:
            ret_l, left_img = left_video.read()
            ret_r, right_img = right_video.read()
            if ret_l is True and ret_r is True:
                marker_loc, disp = self.ptc.run_static_triangulation(left_img, right_img, counter)
                self.marker_location_data.add_vector_entry(counter, marker_loc)

                cv.imshow("disp", disp)
                cv.waitKey(1)
            else:
                break
            counter += 1
        self.marker_location_data.save_data_collected()
        # self.marker_data.open_data(self.marker_data.stream_data_path)
        self.marker_location_data.plot_data()

    def run_through_videos_marker_size(self):
        left_name = self.stream_data_path + 'LeftVideo%dClean.avi' % self.stream_number
        right_name = self.stream_data_path + 'RightVideo%dClean.avi' % self.stream_number
        left_video = cv.VideoCapture(left_name)
        right_video = cv.VideoCapture(right_name)

        counter = 0
        while True:
            ret_l, left_img = left_video.read()
            ret_r, right_img = right_video.read()
            if ret_l is True and ret_r is True:
                # marker_loc, disp = self.ptc.run_static_triangulation(left_img, right_img, counter)
                # self.marker_location_data.add_vector_entry(counter, marker_loc)
                x, y, z, d = self.ptc.run_marker_distance(left_img, right_img, counter)
                print(x, y, z, d)
                self.marker_location_data.add_entry(x, y, z, d, counter)

                # cv.imshow("disp", disp)
                # cv.waitKey(1)
            else:
                break
            counter += 1
        # self.marker_location_data.save_data_collected()
        # self.marker_data.open_data(self.marker_data.stream_data_path)
        print(np.mean(np.array(self.marker_location_data.X)))
        print(np.mean(np.array(self.marker_location_data.Y)))
        print(np.mean(np.array(self.marker_location_data.Z)))
        print(np.mean(np.array(self.marker_location_data.d)))
        self.marker_location_data.plot_data()

    def run_through_videos_dynamic(self):
        left_name = self.stream_data_path + 'LeftVideo%dClean.avi' % self.stream_number
        right_name = self.stream_data_path + 'RightVideo%dClean.avi' % self.stream_number
        left_video = cv.VideoCapture(left_name)
        right_video = cv.VideoCapture(right_name)

        counter = 0
        while True:
            ret_l, left_img = left_video.read()
            ret_r, right_img = right_video.read()
            if ret_l is True and ret_r is True:
                print("Dynamic stream: %d" % counter)
                marker_loc, cam_loc, disp = self.ptc.run_pose_triangulation(left_img, right_img, counter)
                # print(marker_loc)
                self.marker_location_data.add_vector_entry(counter, marker_loc)
                # print(marker_loc)
                self.camera_location_data.add_vector_entry(counter, cam_loc)

                # print(cam_loc)

                cv.imshow("disp", disp)
                cv.waitKey(1)
                # cv.waitKey(0)
            else:
                break
            counter += 1
        self.marker_location_data.save_data_collected()
        # note that the data must be deleted otherwise it will overwrite the location data
        # self.camera_location_data.save_data_collected()
        # self.marker_data.open_data(self.marker_data.stream_data_path)
        self.marker_location_data.condition_data_set(200)
        self.marker_location_data.plot_data()
        # self.camera_location_data.plot_data()

    def run_through_imgs(self):
        left_pathname = self.stream_data_path + 'Left*.jpg'
        right_pathname = self.stream_data_path + 'Right*.jpg'
        left_imgs = list(sorted(glob.glob(left_pathname)))
        right_imgs = list(sorted(glob.glob(right_pathname)))
        assert len(left_imgs) == len(right_imgs)
        print("There are %d images to run through" % len(left_imgs))
        counter = 0
        for left_img_path, right_img_path in zip(left_imgs, right_imgs):
            left_img = cv.imread(left_img_path)
            right_img = cv.imread(right_img_path)

            marker_pt, disp, ret = self.ptc.run_static_triangulation(left_img, right_img, counter)

            # disp, ret = self.ptc.extract_marker_data(left_img, right_img, counter=counter)
            cv.imshow("disp", disp)
            cv.waitKey(1)

            counter += 1
        self.data.save_data_collected()


class TriangulationCalculation:
    def __init__(self):
        self.calib = Calibration.CameraCalibrationData()
        self.tracking_aruco_id = 0
        self.reference_id = 0

        self.left_img = np.array(0)
        self.right_img = np.array(0)
        self.counter = 0
        self.return_val = 1

        self.relative_m_location = np.zeros((3, 1)) # marker location relative to camera
        self.global_marker_location = np.zeros((3, 1)) # global marker location

        self.camera_global_location = np.zeros((3, 1)) # where the camera is
        self.camera_global_rotation = np.zeros((3, 3)) # how the camera is orientated

        self.camera_local_rotation = np.zeros((3, 3)) # camera location relative to marker
        self.camera_local_translation = np.zeros((3, 1)) # camera rotation relative to marker

        self.ret_img = np.array(0)

        self.left_ids = []
        self.left_corners = []
        self.right_ids = []
        self.right_corners = []

        self.cam_rotation_ref = np.zeros((3, 3))
        self.cam_translation_ref = np.zeros((3, 1))
        self.marker_location_ref = np.zeros((3, 1))

    def set_parameters(self, calib_dp, image_size, tracking_id, ref_id):
        self.tracking_aruco_id = tracking_id
        self.reference_id = ref_id
        self.calib.set_parameters(calib_dp, image_size)

    def run_pose_triangulation(self, left_img, right_img, counter):
        # print("Pose Triangulation: %d" % counter)
        self.return_val = 1 # reset value
        self.counter = counter
        self.left_img = cv.remap(left_img, self.calib.maps_left[0], self.calib.maps_left[1], cv.INTER_LINEAR)
        self.right_img = cv.remap(right_img, self.calib.maps_right[0], self.calib.maps_right[1], cv.INTER_LINEAR)

        self.check_image_for_markers()
        self.decide_positioning()
        self.localise_camera()
        self.relative_m_location = self.find_3D_marker_point(self.tracking_aruco_id)
        self.relocate_marker()
        self.localise_marker()
        self.format_dynamic_ret_img()

        # print("global camera and marker locations")
        # print(self.camera_global_location)
        # print(self.global_marker_location)
        # self.camera_translation_data.add_vector_entry(counter, self.camera_location)
        if self.return_val == 0:
            self.global_marker_location = np.array([0, 0, 0]).T
            self.camera_global_location = np.array([0, 0, 0]).T
        return self.global_marker_location, self.camera_global_location, self.ret_img

    def run_static_triangulation(self, left_img, right_img, counter):
        # print("Standard Triangulation: %d" % counter)
        self.return_val = 1  # reset value
        self.counter = counter
        self.left_img = cv.remap(left_img, self.calib.maps_left[0], self.calib.maps_left[1], cv.INTER_LINEAR)
        self.right_img = cv.remap(right_img, self.calib.maps_right[0], self.calib.maps_right[1], cv.INTER_LINEAR)

        self.check_image_for_markers()
        self.global_marker_location = self.find_3D_marker_point(self.tracking_aruco_id)
        self.localise_marker()
        self.format_ret_img()

        if self.return_val == 0:
            self.global_marker_location = np.array([0, 0, 0]).T
        return self.global_marker_location, self.ret_img

    def run_marker_distance(self, left_img, right_img, counter):
        self.return_val = 1  # reset value
        self.counter = counter
        self.left_img = cv.remap(left_img, self.calib.maps_left[0], self.calib.maps_left[1], cv.INTER_LINEAR)
        self.right_img = cv.remap(right_img, self.calib.maps_right[0], self.calib.maps_right[1], cv.INTER_LINEAR)

        self.check_image_for_markers()
        x, y, z, d = self.determine_aruco_distance()
        return x, y, z, d

    def localise_camera(self):
        if self.cam_translation_ref[0] == 0:
            self.cam_rotation_ref = self.camera_local_rotation
            self.cam_translation_ref = self.camera_local_translation
            print("Setting camera localisation values as: (translation)")
            print(self.cam_translation_ref)

        try:
            rot_inv = np.linalg.inv(np.array(self.cam_rotation_ref))
        except:
            rot_inv = np.identity(3)
        self.camera_global_rotation = rot_inv.dot(np.array(self.camera_local_rotation))

        global_translation_offset = self.camera_global_rotation.dot(self.camera_local_translation)
        self.camera_global_location = global_translation_offset - self.cam_translation_ref

        # self.camera_data.add_vector_entry(self.counter, self.camera_global_location)

    def get_camera_position_single_image(self, side):
        if side == "right":
            corners = self.right_corners
            cam_mtx = self.calib.cam_mtx_right
            dist = self.calib.dist_right
            ids = self.right_ids
            img = self.right_img
        elif side == "left":
            corners = self.left_corners
            cam_mtx = self.calib.cam_mtx_left
            dist = self.calib.dist_left
            ids = self.left_ids
            img = self.left_img
        else:
            print("Invalid argument for side")
            return 0

        if ids is not None:
            rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners, 10, cam_mtx, dist)
            try:
                i = np.where(ids == self.reference_id)
                i = i[0][0]

                rotation = rvecs[i]
                translation = tvecs[i]
                aruco.drawAxis(img, cam_mtx, dist, rotation, translation, 14)
                # print(rotation)
                rotation, jac = cv.Rodrigues(rotation)
                translation = np.array(translation).T
            except:
                print("ID not found")
                rotation = [0, 0, 0]
                translation = [0, 0, 0]
                self.return_val = 0
            return translation, rotation
            # self.camera_local_translation = translation
            # self.camera_local_rotation = rotation
        else:
            self.return_val = 0
            print("No Aruco marker to locate from")
            return 0, 0

    def find_specific_marker_centers(self, id_to_find):
        corner_list = [self.left_corners, self.right_corners]
        id_list = [self.left_ids, self.right_ids]
        centers = []
        ret = True
        for ids, corners in zip(id_list, corner_list):
            try:
                i = np.where(ids == id_to_find)
                i = i[0][0]
                height = (corners[i][0, 0, 0] + corners[i][0, 1, 0] + corners[i][0, 2, 0] + corners[i][0, 3, 0]) / 4
                width = (corners[i][0, 0, 1] + corners[i][0, 1, 1] + corners[i][0, 2, 1] + corners[i][0, 3, 1]) / 4
                centers.append((height, width))
            except:
                ret = False # if the id or corners aren't found in an image
                centers.append((0, 0))

        center_left = centers[0]
        center_right = centers[1]
        return center_left, center_right, ret

    def find_3D_marker_point(self, id):
        point_left, point_right, center_return = self.find_specific_marker_centers(id)

        if center_return == 1:
            homogeneous_point = cv.triangulatePoints(self.calib.projLft, self.calib.projRgt, point_left, point_right)

            point3D = homogeneous_point / homogeneous_point[3]

            cv.circle(self.left_img, (int(point_left[0]), int(point_left[1])), 5, (0, 0, 255))
            cv.circle(self.right_img, (int(point_right[0]), int(point_right[1])), 5, (0, 0, 255))

            return np.array(point3D[:3])
        else:
            print("Problem encountered in finding three d point: %d" % self.counter)
            self.return_val = 0
            return np.array([0, 0, 0])

    def localise_marker(self):
         # check if local is empty
        if self.return_val == 0:
             self.global_marker_location = [0, 0, 0]
             print("No point triangulated")
        else:
            if self.marker_location_ref[0] == 0:
                print(self.marker_location_ref)
                self.marker_location_ref[0] = self.global_marker_location[0]
                self.marker_location_ref[1] = self.global_marker_location[1]
                self.marker_location_ref[2] = self.global_marker_location[2]
                print("Marker localisation location is set as")
                print(self.marker_location_ref)

            self.global_marker_location[0] = self.global_marker_location[0] - self.marker_location_ref[0]
            self.global_marker_location[1] = self.global_marker_location[1] - self.marker_location_ref[1]
            self.global_marker_location[2] = self.global_marker_location[2] - self.marker_location_ref[2]

    def format_ret_img(self):
        counter = self.counter

        disp_img = np.concatenate((self.left_img, self.right_img), axis=1)
        size = (int(disp_img.shape[1]), int(disp_img.shape[0]))
        disp_img = cv.resize(disp_img, size)

        x = int(self.global_marker_location[0])
        y = int(self.global_marker_location[1])
        z = int(self.global_marker_location[2])
        d = (x ** 2 + y ** 2 + z ** 2) ** 0.5

        font = cv.FONT_HERSHEY_TRIPLEX
        xLocation = 600
        cv.rectangle(disp_img, (600, 30), (800, 280), (255, 255, 255), -1)
        cv.putText(disp_img, 'X:' + str(x), (xLocation, 100), font, 1, (0, 0, 0))
        cv.putText(disp_img, 'Y:' + str(y), (xLocation, 160), font, 1, (0, 0, 0))
        cv.putText(disp_img, 'Z:' + str(z), (xLocation, 220), font, 1, (0, 0, 0))
        cv.putText(disp_img, 'd:' + str(int(d)), (xLocation, 280), font, 2, (0, 0, 0))
        cv.putText(disp_img, 'Time: ' + str(counter), (xLocation, 350), font, 2, (0, 0, 255))

        self.ret_img = disp_img

    def format_dynamic_ret_img(self):
        counter = self.counter

        disp_img = np.concatenate((self.left_img, self.right_img), axis=1)
        size = (int(disp_img.shape[1]), int(disp_img.shape[0]))
        disp_img = cv.resize(disp_img, size)

        x = int(self.global_marker_location[0])
        y = int(self.global_marker_location[1])
        z = int(self.global_marker_location[2])
        d = (x ** 2 + y ** 2 + z ** 2) ** 0.5

        x_c = int(self.camera_global_location[0][0])
        y_c = int(self.camera_global_location[1][0])
        z_c = int(self.camera_global_location[2][0])
        d_c = (x_c ** 2 + y_c ** 2 + z_c ** 2) ** 0.5

        font = cv.FONT_HERSHEY_TRIPLEX
        xLocation = 600
        cv.rectangle(disp_img, (500, 30), (800, 280), (255, 255, 255), -1)
        cv.putText(disp_img, 'X:' + str(x) + " : " + str(x_c), (xLocation, 100), font, 1, (0, 0, 0))
        cv.putText(disp_img, 'Y:' + str(y) + " : " + str(y_c), (xLocation, 160), font, 1, (0, 0, 0))
        cv.putText(disp_img, 'Z:' + str(z) + " : " + str(z_c), (xLocation, 220), font, 1, (0, 0, 0))
        cv.putText(disp_img, 'd:' + str(int(d)) + " : " + str(d_c), (xLocation, 280), font, 2, (0, 0, 0))
        cv.putText(disp_img, 'Time: ' + str(counter), (xLocation, 350), font, 2, (0, 0, 255))

        self.ret_img = disp_img

    def relocate_marker(self):
        global_coord_m_loc = np.array(self.camera_local_rotation).dot(self.relative_m_location)
        gml = self.camera_global_location - global_coord_m_loc
        for i in range(3):
            self.global_marker_location[i] = gml[i][0]
        # print("marker location at camera centre in global coord")
        # print(global_coord_m_loc)
        # print("Camera location at global O in global coord")
        # print(self.camera_location)
        # print("marker location at global O in global coord")
        # print(self.global_marker_location)

    def stereo_pose_camera_rotation(self):
        # set the camera rotation from stereo
        trans, rotation_left = self.get_camera_position_single_image("left")
        trans, rotation_right = self.get_camera_position_single_image("right")
        rotation = (rotation_left + rotation_right)/2
        self.camera_local_rotation = rotation

    def check_image_for_markers(self):
        aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_100)

        self.left_corners, self.left_ids, _ = aruco.detectMarkers(self.left_img, aruco_dict)
        self.right_corners, self.right_ids, _ = aruco.detectMarkers(self.right_img, aruco_dict)

        if (not self.left_corners) or (not self.right_corners):
            print("No markers found")
            self.return_val = 0
            return 0

        aruco.drawDetectedMarkers(self.left_img, self.left_corners, self.left_ids)
        aruco.drawDetectedMarkers(self.right_img, self.right_corners, self.right_ids)
        return 1

    def decide_positioning(self):
        center_left, center_right, center_return = self.find_specific_marker_centers(self.reference_id)
        if center_return is True:
            # this means marker is in both images
            print("Using dual cam positioning")
            self.camera_local_translation = self.find_3D_marker_point(self.reference_id)
            self.stereo_pose_camera_rotation()

        # these functions will use a single marker position to try and get the camera locaiton
        elif center_left != (0, 0):
            print("Using left cam positioning: " + str(center_left))
            self.camera_local_translation, self.camera_local_rotation = self.get_camera_position_single_image("left")
        elif center_right != (0, 0):
            print("Using right cam positioning: " + str(center_right))
            self.camera_local_translation, self.camera_local_rotation = self.get_camera_position_single_image("right")
        else:
            # no markers found
            self.return_val = 0

    def determine_aruco_distance(self):
        corners_right = self.right_corners
        corners_left = self.left_corners

        if not corners_left or not corners_right:
            pass
        else:
            left_top_left = corners_left[0][0, 0, :]
            left_top_right = corners_left[0][0, 1, :]
            left_bot_right = corners_left[0][0, 2, :]
            left_bot_left = corners_left[0][0, 3, :]

            right_top_left = corners_right[0][0, 0, :]
            right_top_right = corners_right[0][0, 1, :]
            right_bot_right = corners_right[0][0, 2, :]
            right_bot_left = corners_right[0][0, 3, :]

            # print(left_top_left, right_top_left)
            # print(left_top_right, right_top_right)
            # print(left_bot_left, right_bot_left)
            # print(left_bot_right, right_bot_right)

            homogeneous_point = cv.triangulatePoints(self.calib.projLft, self.calib.projRgt, left_top_left, right_top_left)
            point3D_top_left = homogeneous_point / homogeneous_point[3]
            homogeneous_point = cv.triangulatePoints(self.calib.projLft, self.calib.projRgt, left_top_right, right_top_right)
            point3D_top_right = homogeneous_point / homogeneous_point[3]
            homogeneous_point = cv.triangulatePoints(self.calib.projLft, self.calib.projRgt, left_bot_left, right_bot_left)
            point3D_bot_left = homogeneous_point / homogeneous_point[3]
            homogeneous_point = cv.triangulatePoints(self.calib.projLft, self.calib.projRgt, left_bot_right, right_bot_right)
            point3D_bot_right = homogeneous_point / homogeneous_point[3]

            dist_top = point3D_top_right - point3D_top_left
            dist_left = point3D_top_left - point3D_bot_left
            dist_bot = point3D_bot_right - point3D_bot_left
            dist_right = point3D_top_right - point3D_bot_right

            # print("Top point right relative to left")
            # print(point3D_top_right - point3D_top_left)

            print("Top")
            # print(dist_top)
            # print(np.linalg.norm(dist_top))
            print("Bottom")
            # print(dist_bot)
            # print(np.linalg.norm(dist_bot))
            print("Left")
            # print(dist_left)
            # print(np.linalg.norm(dist_left))
            print("Right")
            # print(dist_right)
            # print(np.linalg.norm(dist_right))
            # print("Distances")
            # print()
            x = np.linalg.norm(dist_top)
            y = np.linalg.norm(dist_bot)
            z = np.linalg.norm(dist_left)
            d = np.linalg.norm(dist_right)

            return x, y, z, d
