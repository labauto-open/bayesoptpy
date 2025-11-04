#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# memo
# - Windows PowerShell is supposed to be used

import os
import pyautogui as pag
import subprocess
import sys
import time
import win32api
import win32gui
import win32con


class GUIControlWindows():
    def __init__(self, window_name, exe_dir, exe_cmd, exe_sleep):
        # variables
        self.active_window_x1 = 0
        self.active_window_y1 = 0
        self.active_window_x2 = 0
        self.active_window_y2 = 0
        self.active_window_width = 0
        self.active_window_length = 0
        self.active_window_name = ''
        self.screen_x,self.screen_y = pag.size()
        print('screen size:', self.screen_x, self.screen_y)

        self.window_name = window_name
        self.exe_dir = exe_dir
        self.exe_cmd = exe_cmd
        self.exe_sleep = exe_sleep


    def execute_application(self):
        self.parent_handle = win32gui.FindWindow(None, self.window_name)  # check whether software has already been running
        if self.parent_handle > 0:
            print('software has been already running')
        else:
            os.chdir(self.exe_dir)
            subprocess.Popen(self.exe_cmd, shell=True)
            time.sleep(self.exe_sleep)


    def make_window_active(self, window_name):
        # try
        self.parent_handle = win32gui.FindWindow(None, window_name)
        # check
        if self.parent_handle == 0:
            print('Fail to get window', window_name)
            sys.exit()
        elif self.parent_handle > 0 :
            print('Active window has changed to ', window_name)
            # get window position
            self.active_window_x1,self.active_window_y1,self.active_window_x2,self.active_window_y2 = win32gui.GetWindowRect(self.parent_handle)
            self.active_window_width = self.active_window_x2 - self.active_window_x1
            self.active_window_length = self.active_window_y2 - self.active_window_y1
            print('window position:', self.active_window_x1, self.active_window_y1)
            print('window size:', self.active_window_width, self.active_window_length)

            # make window foreground
            win32gui.SetForegroundWindow(self.parent_handle)

            # get window info
            self.active_window_name = win32gui.GetWindowText(self.parent_handle)
            classname = win32gui.GetClassName(self.parent_handle)

        else:
            print('something wrong')


    def click_by_pos(self, offset_x, offset_y, clicks=1, sleep_time=0.5, duration=1, relative=None):
        '''
        - default (relative None): click offset_x,y position of the active window
        - relative (relative 1)  : click offset_x,y position from the current mouse position
        '''
        x_current, y_current = pag.position()
        if relative is None:
            pag.moveTo(self.active_window_x1+offset_x, self.active_window_y1+offset_y, duration)
        else:
            pag.moveTo(x_current+offset_x, y_current+offset_y, duration)

        x, y = pag.position()
        print('click position:', x, y)
        pag.click(x, y, clicks)
        
        time.sleep(sleep_time)


    def click_by_pos_on_window(self, window_name, offset_x, offset_y, clicks):
        # arg
        # - window_name: the window to be active
        # - offset_x, y : clickする座標. windowの左端からのpos
        # - clicks: number of click
        self.make_window_active(window_name)
        print('%s window is activated' % window_name)
        self.click_by_pos(offset_x, offset_y, clicks)
        print('click position: (%s, %s). %s times' % (offset_x, offset_y, clicks))


    def click_by_img(self, img_path, clicks=1, conf=0.9, sleep_time=1):
        print('img_path:', img_path)

        x,y = pag.locateCenterOnScreen(img_path, confidence=conf)
        print('click position:', x, y)
        pag.click(x, y, clicks)

        time.sleep(sleep_time)

        print(self.active_window_name)


    def search_window_by_img(self, img_path, duration=1):
        x,y = pag.locateOnScreen(img_path)
        pag.moveTo(x, y, duration)
        

    def get_window_name(self):
        '''
        return foreground window name
        '''
        handle = win32gui.GetForegroundWindow()
        window_name = win32gui.GetWindowText(handle)
        print(window_name)

        return window_name


    def get_active_window_name(self):
        return self.active_window_name
