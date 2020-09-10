import cv2 as cv
import numpy as np
import os
import glob


class RunCalibration:
    # this deals with the image stream and produces a camera calibration data object
    def __init__(self):
        self.session_path = ""
        self.calibration_data_path = ""
        self.pattern_size = (8, 6)
        self.left_pts = []
        self.right_pts = []
        self.scaling_factor = 4  # should convert to cm from dimensions used
        self.image_size = (0, 0)
        self.distance = []

        self.calib_data = CameraCalibrationData()

    def run_calibration(self, session, calibration, image_size=(640, 480)):
        self.session_path = 'StaticServerSession%d/' % session
        self.calibration_data_path = self.session_path + "staticCalibrationData%d/" % calibration
        self.image_size = image_size
        self.calib_data.calibration_data_path = self.calibration_data_path
        self.calib_data.image_size = image_size

        self.run_through_imgs()
        self.compute_stereo_calibration()
        self.calib_data.calculate_projection_mtxs()
        self.calib_data.create_maps()

    def run_through_imgs(self):
        left_pathname = self.calibration_data_path + 'Left*.jpg'
        right_pathname = self.calibration_data_path + 'Right*.jpg'
        left_imgs = list(sorted(glob.glob(left_pathname)))
        right_imgs = list(sorted(glob.glob(right_pathname)))
        assert len(left_imgs) == len(right_imgs)
        print("There are %d images to run through" % len(left_imgs))
        counter = 0
        for left_img_path, right_img_path in zip(left_imgs, right_imgs):
            left_img = cv.imread(left_img_path)
            right_img = cv.imread(right_img_path)
            ret_val = self.find_chessboard_corners(left_img, right_img)

            if ret_val == 0:
                print("Corners not found (%d): removing images" % counter)
                os.remove(left_img_path)
                os.remove(right_img_path)
            counter += 1
        cv.destroyAllWindows()

    def find_chessboard_corners(self, left_img, right_img):
        try:
            img_gray_right = cv.cvtColor(right_img, cv.COLOR_RGB2GRAY)
            img_gray_left = cv.cvtColor(left_img, cv.COLOR_RGB2GRAY)
        except:
            img_gray_right = right_img
            img_gray_left = left_img
        criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 1e-3)
        img_size = None
        if img_size is None:
            img_size = (left_img.shape[1], left_img.shape[0])
            self.image_size = img_size

        res_left, corners_left = cv.findChessboardCorners(img_gray_left, self.pattern_size)
        res_right, corners_right = cv.findChessboardCorners(img_gray_right, self.pattern_size)

        if res_left == 1 and res_right == 1:
            corners_left = cv.cornerSubPix(img_gray_left, corners_left, (10, 10), (-1, -1), criteria)
            self.left_pts.append(corners_left)
            corners_right = cv.cornerSubPix(img_gray_right, corners_right, (10, 10), (-1, -1), criteria)
            self.right_pts.append(corners_right)

            cv.drawChessboardCorners(left_img, self.pattern_size, corners_left, res_left)
            cv.drawChessboardCorners(right_img, self.pattern_size, corners_right, res_right)
            disp_img = np.concatenate((left_img, right_img), axis=1)
            cv.imshow("chess", disp_img)
            cv.waitKey(1)

            return 1
        else:
            return 0

    def compute_stereo_calibration(self):
        pattern_points = np.zeros((np.prod(self.pattern_size), 3), np.float32)
        pattern_points[:, :2] = np.indices(self.pattern_size).T.reshape(-1, 2)
        pattern_points = pattern_points * self.scaling_factor
        pattern_points = [pattern_points] * len(self.left_pts)

        img_size = tuple(self.image_size)

        err, Kl, Dl, Kr, Dr, R, T, E, F = cv.stereoCalibrate(pattern_points, self.left_pts, self.right_pts, None, None,
                                                             None, None, img_size, flags=0)
        # note that the error, essential and fundemental matrices are not stored
        self.calib_data.cam_mtx_left = Kl
        self.calib_data.cam_mtx_right = Kr
        self.calib_data.dist_left = Dl
        self.calib_data.dist_right = Dr
        self.calib_data.ref_rotation = R
        self.calib_data.ref_translation = T

        x = T[0]
        y = T[1]
        z = T[2]
        dis = (x ** 2 + y ** 2 + z ** 2) ** 0.5
        print("Translation matrix")
        print(T)
        print("Distance betwen Camera's: " + str(dis))


class CameraCalibrationData:
    def __init__(self):
        self.projLft = np.zeros((3, 4))
        self.projRgt = np.zeros((3, 4))
        self.cam_mtx_left = np.zeros((3, 3))
        self.cam_mtx_right = np.zeros((3, 3))
        self.dist_left = []
        self.dist_right = []
        self.rotation_left = []
        self.rotation_right = []
        self.ref_rotation = []
        self.ref_translation = []
        # maps left and right will each have two maps that are used to remap the functions
        self.maps_left = []
        self.maps_right = []

        self.image_size = (0, 0)
        self.calibration_data_path = ""

    def set_parameters(self, calibration_dp, image_size=(640, 480)):
        self.image_size = image_size
        self.calibration_data_path = calibration_dp

        self.load_calib_data()
        self.create_maps()

    def load_calib_data(self):
        self.projLft = np.load(self.calibration_data_path + 'projLeft.npy')
        self.projRgt = np.load(self.calibration_data_path + 'projRight.npy')
        self.cam_mtx_left = np.load(self.calibration_data_path + 'mtxLeft.npy')
        self.cam_mtx_right = np.load(self.calibration_data_path + 'mtxRgt.npy')
        self.dist_left = np.load(self.calibration_data_path + 'distLft.npy')
        self.dist_right = np.load(self.calibration_data_path + 'distRgt.npy')
        self.rotation_left = np.load(self.calibration_data_path + 'RotationLeft.npy')
        self.rotation_right = np.load(self.calibration_data_path + 'RotationRight.npy')

    def calculate_projection_mtxs(self):
        r_left = np.zeros((3, 3))
        r_right = np.zeros((3, 3))
        proj_left = np.zeros((3, 4))
        proj_right = np.zeros((3, 4))

        cv.stereoRectify(self.cam_mtx_left, self.dist_left, self.cam_mtx_right, self.dist_right, self.image_size,
                         self.ref_rotation, self.ref_translation, r_left, r_right, proj_left, proj_right)

        self.projLft = proj_left
        self.projRgt = proj_right
        self.rotation_right = r_right
        self.rotation_left = r_left

        self.save_calib_data()

        print("Projection matrices computed:")
        print(proj_left)
        print(proj_right)

    def save_calib_data(self):
        np.save(self.calibration_data_path + '/projLeft', self.projLft)
        np.save(self.calibration_data_path + '/projRight', self.projRgt)
        np.save(self.calibration_data_path + '/Rotationleft', self.rotation_left)
        np.save(self.calibration_data_path + '/RotationRight', self.rotation_right)

        np.save(self.calibration_data_path + '/mtxLeft', self.cam_mtx_left)
        np.save(self.calibration_data_path + '/mtxRgt', self.cam_mtx_right)
        np.save(self.calibration_data_path + '/distLft', self.dist_left)
        np.save(self.calibration_data_path + '/distRgt', self.dist_right)
        np.save(self.calibration_data_path + '/rotation', self.ref_rotation)
        np.save(self.calibration_data_path + '/translation', self.ref_translation)

    def create_maps(self):
        map_1, map_2 = cv.initUndistortRectifyMap(self.cam_mtx_left, self.dist_left, self.rotation_left, self.projLft,
                                                  self.image_size, cv.CV_32FC1)
        self.maps_left.append(map_1)
        self.maps_left.append(map_2)
        map_1, map_2 = cv.initUndistortRectifyMap(self.cam_mtx_right, self.dist_right, self.rotation_right, self.projLft,
                                                  self.image_size, cv.CV_32FC1)
        self.maps_right.append(map_1)
        self.maps_right.append(map_2)






