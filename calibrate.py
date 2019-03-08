#!/usr/bin/env python
# -*- coding: utf-8 -*-

# BASED ON CODE FROM https://github.com/mbeyeler/opencv-python-blueprints/tree/master/chapter4

"""A module for camera calibration using a chessboard"""

import cv2
import numpy as np
import wx

#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""A module containing simple GUI layouts using wxPython"""

import abc
import six
import time


__author__ = "Michael Beyeler"
__license__ = "GNU GPL 3.0 or later"


class BaseLayout(wx.Frame):
    """Abstract base class for all layouts

        A custom layout needs to implement at least three methods:
        * _init_custom_layout:   A method to initialize all relevant
                                 parameters. This method will be called in the
                                 class constructor, after initializing common
                                 parameters, right before creating the GUI
                                 layout.
        * _create_custom_layout: A method to create a custom GUI layout. This
                                 method will be called in the class
                                 constructor, after initializing common
                                 parameters.
                                 Every GUI contains the camera feed in the
                                 variable self.pnl.
                                 Additional layout elements can be added below
                                 the camera feed by means of the method
                                 self.panels_vertical.Add.
        * _process_frame:        A method to process the current RGB camera
                                 frame. It needs to return the processed RGB
                                 frame to be displayed.
    """

    def __init__(self, capture, title=None, parent=None, id=-1, fps=10):
        """Class constructor

            This method initializes all necessary parameters and generates a
            basic GUI layout that can then be modified by
            self.init_custom_layout() and self.create_custom_layout().

            :param parent: A wx.Frame parent (often Null). If it is non-Null,
                the frame will be minimized when its parent is minimized and
                restored when it is restored.
            :param id: The window identifier. Value -1 indicates default value.
            :param title: The caption to be displayed on the frame's title bar.
            :param capture: A cv2.VideoCapture object to be used as camera
                feed.
            :param fps: frames per second at which to display camera feed
        """
        self.capture = capture
        self.fps = fps

        # determine window size and init wx.Frame
        success, frame = self._acquire_frame()
        if not success:
            print ("Could not acquire frame from camera.")
            raise SystemExit

        self.imgHeight, self.imgWidth = frame.shape[:2]
        self.bmp = wx.BitmapFromBuffer(self.imgWidth, self.imgHeight, frame)
        wx.Frame.__init__(self, parent, id, title,
                          size=(self.imgWidth, self.imgHeight))

        self._init_base_layout()
        self._create_base_layout()

    def _init_base_layout(self):
        """Initialize parameters

            This method performs initializations that are common to all GUIs,
            such as the setting up of a timer.

            It then calls an abstract method self.init_custom_layout() that
            allows for additional, application-specific initializations.
        """
        # set up periodic screen capture
        self.timer = wx.Timer(self)
        self.timer.Start(1000. / self.fps)
        self.Bind(wx.EVT_TIMER, self._on_next_frame)

        # allow for custom modifications
        self._init_custom_layout()

    def _create_base_layout(self):
        """Create generic layout

            This method sets up a basic layout that is common to all GUIs, such
            as a live stream of the camera (capture device). This stream is
            assigned to the variable self.pnl, and arranged in a vertical
            layout self.panels_vertical.

            Additional layout elements can be added below the livestream by
            means of the method self.panels_vertical.Add.
        """
        # set up video stream
        self.pnl = wx.Panel(self, size=(self.imgWidth, self.imgHeight))
        self.pnl.SetBackgroundColour(wx.BLACK)
        self.pnl.Bind(wx.EVT_PAINT, self._on_paint)

        # display the button layout beneath the video stream
        self.panels_vertical = wx.BoxSizer(wx.VERTICAL)
        self.panels_vertical.Add(self.pnl, 1, flag=wx.EXPAND | wx.TOP,
                                 border=1)

        # allow for custom layout modifications
        self._create_custom_layout()

        # round off the layout by expanding and centering
        self.SetMinSize((self.imgWidth, self.imgHeight))
        self.SetSizer(self.panels_vertical)
        self.Centre()

    def _on_next_frame(self, event):
        """
            This method captures a new frame from the camera (or capture
            device) and sends an RGB version to the method self.process_frame.
            The latter will then apply task-specific post-processing and return
            an image to be displayed.
        """
        success, frame = self._acquire_frame()
        if success:
            # process current frame
            frame = self._process_frame(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))

            # update buffer and paint (EVT_PAINT triggered by Refresh)
            self.bmp.CopyFromBuffer(frame)
            cv2.imshow("frame", frame)
            self.Refresh(eraseBackground=False)
        else:
            print("rip")

    def _on_paint(self, event):
        """
            This method draws the camera frame stored in the bitmap self.bmp
            onto the panel self.pnl. Make sure self.pnl exists and is at least
            the size of the camera frame.
            This method is called whenever an event wx.EVT_PAINT is triggered.
        """
        # read and draw buffered bitmap
        deviceContext = wx.BufferedPaintDC(self.pnl)
        deviceContext.DrawBitmap(self.bmp, 0, 0)

    def _acquire_frame(self):
        """
            This method is called whenever a new frame needs to be acquired.
            :returns: (success, frame), whether acquiring was successful
                      (via Boolean success) and current frame
        """
        return self.capture.read()

    @abc.abstractmethod
    def _init_custom_layout(self):
        """
            This method is called in the class constructor, after setting up
            relevant event callbacks, and right before creation of the GUI
            layout.
        """
        pass

    @abc.abstractmethod
    def _create_custom_layout(self):
        """
            This method is responsible for creating the GUI layout.
            It is called in the class constructor, after setting up relevant
            event callbacks and self.init_layout, and creates the layout.
            Every GUI contains the camera feed in the variable self.pnl.
            Additional layout elements can be added below the camera feed by
            adding them to self.panels_vertical.
        """
        pass

    @abc.abstractmethod
    def _process_frame(self, frame_rgb):
        """
            This method is responsible for any post-processing that needs to be
            applied to the current frame of the camera (capture device) stream.

            :param frame_rgb: The RGB camera frame to be processed.
            :returns: The processed RGB camera frame to be displayed.
        """
        pass

class CameraCalibration(BaseLayout):
    """Camera calibration
        This class performs camera calibration on a webcam video feed using
        the chessboard approach described here:
        http://docs.opencv.org/doc/tutorials/calib3d/camera_calibration/camera_calibration.html
    """

    def _init_custom_layout(self):
        """Initializes camera calibration"""
        # setting chessboard size
        self.chessboard_size = (9, 6)

        # prepare object points
        self.objp = np.zeros((np.prod(self.chessboard_size), 3),
                             dtype=np.float32)
        self.objp[:, :2] = np.mgrid[0:self.chessboard_size[0],
                                    0:self.chessboard_size[1]].T.reshape(-1, 2)

        # prepare recording
        self.recording = False
        self.record_min_num_frames = 20
        self._reset_recording()

    def _create_custom_layout(self):
        """Creates a horizontal layout with a single button"""
        pnl = wx.Panel(self, -1)
        self.button_calibrate = wx.Button(pnl, label='Calibrate Camera')
        self.Bind(wx.EVT_BUTTON, self._on_button_calibrate)
        hbox = wx.BoxSizer(wx.HORIZONTAL)
        hbox.Add(self.button_calibrate)
        pnl.SetSizer(hbox)

        self.panels_vertical.Add(pnl, flag=wx.EXPAND | wx.BOTTOM | wx.TOP,
                                 border=1)

    def _process_frame(self, frame):
        """Processes each frame
            If recording mode is on (self.recording==True), this method will
            perform all the hard work of the camera calibration process:
            - for every frame, until enough frames have been processed:
                - find the chessboard corners
                - refine the coordinates of the detected corners
            - after enough frames have been processed:
                - estimate the intrinsic camera matrix and distortion
                  coefficients
            :param frame: current RGB video frame
            :returns: annotated video frame showing detected chessboard corners
        """
        # if we are not recording, just display the frame
        if not self.recording:
            return frame

        # else we're recording
        img_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY).astype(np.uint8)
        if self.record_cnt < self.record_min_num_frames:
            # need at least some number of chessboard samples before we can
            # calculate the intrinsic matrix
            ret, corners = cv2.findChessboardCorners(img_gray,
                                                     self.chessboard_size,
                                                     None)
            print("lol")
            if ret:
                cv2.drawChessboardCorners(frame, self.chessboard_size, corners,
                                          ret)

                # refine found corners
                criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
                            30, 0.01)
                cv2.cornerSubPix(img_gray, corners, (9, 9), (-1, -1), criteria)

                self.obj_points.append(self.objp)
                self.img_points.append(corners)
                self.record_cnt += 1
                print("good")

        else:
            # we have already collected enough frames, so now we want to
            # calculate the intrinsic camera matrix (K) and the distortion
            # vector (dist)
            print("Calibrating...")
            ret, K, dist, rvecs, tvecs = cv2.calibrateCamera(self.obj_points,
                                                             self.img_points,
                                                             (self.imgHeight,
                                                              self.imgWidth),
                                                             None, None)
            print("K=", K)
            print("dist=", dist)

            data = {"camera_matrix": K.tolist(), "dist_coeff": dist.tolist()}
            fname = "calibration.json"
            import json
            with open(fname, "w") as f:
                json.dump(data, f)

            # double-check reconstruction error (should be as close to zero as
            # possible)
            mean_error = 0
            for i in range(len(self.obj_points)):
                img_points2, _ = cv2.projectPoints(self.obj_points[i],
                                                   rvecs[i], tvecs[i], K, dist)
                error = cv2.norm(self.img_points[i], img_points2,
                                 cv2.NORM_L2) / len(img_points2)
                mean_error += error

            print("mean error=", mean_error)

            self.recording = False
            self._reset_recording()
            self.button_calibrate.Enable()

        return frame

    def _on_button_calibrate(self, event):
        """Enable recording mode upon pushing the button"""
        self.button_calibrate.Disable()
        self.recording = True
        self._reset_recording()

    def _reset_recording(self):
        """Disable recording mode and reset data structures"""
        self.record_cnt = 0
        self.obj_points = []
        self.img_points = []


def main():
    capture = cv2.VideoCapture(2)
    if not(capture.isOpened()):
        capture.open()

    if hasattr(cv2, 'cv'):
        capture.set(cv2.cv.CV_PROP_FOURCC, cv2.cv.CV_FOURCC('M', 'J', 'P', 'G') );
        capture.set(cv2.cv.CV_CAP_PROP_FRAME_WIDTH, 1280)
        capture.set(cv2.cv.CV_CAP_PROP_FRAME_HEIGHT, 960)
    else:
        # capture.set(cv2.CAP_PROP_FOURCC, 4);
        capture.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        capture.set(cv2.CAP_PROP_FRAME_HEIGHT, 960)

    # start graphical user interface
    app = wx.App()
    layout = CameraCalibration(capture, title='Camera Calibration')
    layout.Show(True)
    app.MainLoop()


if __name__ == '__main__':
    main()