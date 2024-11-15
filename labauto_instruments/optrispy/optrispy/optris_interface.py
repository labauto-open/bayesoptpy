#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
from serial_interface import SerialInterface

'''
Memo
- Serial communication is enabled with PIX Connect software (the measurement software for Windows) via COM port. (regardless of the camera??)
- RS485 is supported for Xi80/410 camera, which is effective for embedded or stand-alone usage of the camera without the software. This enables direct communication with a computer where Ubuntu OS is installed.
- Implementation is skipped for the followings so far
  - ?TMA
  - ?TCO
  - set commands
'''

class OptrisInterface(SerialInterface):
    def __init__(self):
        super().__init__(baudrate=115200, timeout=1)


    def connect_optris(self):
        self.open()
        time.sleep(0.5)


    def get_data_by_cmd(self, cmd, decode_error_option='strict'):
        self.write_str(cmd)
        data_raw = self.readline_str(decode_error_option)
        data_header = cmd.replace('?', '!').rstrip()  # change character and remove newline code
        data = data_raw.replace(data_header+'=', '')

        #print('data_raw:', data_raw)
        #print('data', data)

        return data


    def convert_temp_data_to_value(self, temp_data):
        # raw temp_data includes 0xb0 code as default which means 'deg' symbol (small circle)
        return float(temp_data.replace('C', ''))


    def get_temperature(self):
        '''
        Description:
          Read temperature of main measure area
        '''
        cmd = '?T\r\n'
        return self.convert_temp_data_to_value(self.get_data_by_cmd(cmd, 'ignore'))


    def get_temperature_index(self, index):
        'Read temperature of measure area with index i'
        cmd = '?T('+str(index)+')\r\n'

        return self.convert_temp_data_to_value(self.get_data_by_cmd(cmd, 'ignore'))



    def get_chip_temperature(self):
        '''
        Description:
          Read chip temperature
        '''
        cmd = '?C\r\n'

        return self.convert_temp_data_to_value(self.get_data_by_cmd(cmd, 'ignore'))


    def get_flag_temperature(self):
        '''
        Description:
          Read flag temperature
        '''
        cmd = '?F\r\n'

        return self.convert_temp_data_to_value(self.get_data_by_cmd(cmd, 'ignore'))


    def get_internal_temperature(self):
        '''
        Description:
          Read internal temperature
        '''
        cmd = '?I\r\n'

        return self.convert_temp_data_to_value(self.get_data_by_cmd(cmd, 'ignore'))


    def get_emissivity(self):
        '''
        Description:
          Read fixed emissivity value
        '''
        cmd = '?E\r\n'

        return float(self.get_data_by_cmd(cmd))


    def get_transmissivity(self):
        '''
        Description:
          Read fixed transmissivity value
        '''
        cmd = '?XG\r\n'

        return float(self.get_data_by_cmd(cmd))


    def get_ambient_temperature(self):
        '''
        Description:
          Read fixed ambient temperature value
        '''
        cmd = '?A\r\n'

        return self.convert_temp_data_to_value(self.get_data_by_cmd(cmd, 'ignore'))


    def get_serial_number(self):
        '''
        Description:
          Read serial number of imager
        '''
        cmd = '?SN\r\n'

        return int(self.get_data_by_cmd(cmd))


    def get_measure_area_count(self):
        '''
        Description:
          Read count of measure areas
        '''
        cmd = '?AreaCount\r\n'

        return int(self.get_data_by_cmd(cmd))


    def get_area_mode(self, index):
        '''
        Description:
          Read mode ID of area i'
        Return:
          Id: 0(Min), 1(Max), 2(Average), 3(Distribution)
        '''
        cmd = '?AreaMode('+str(index)+')\r\n'

        return int(self.get_data_by_cmd(cmd))


    def get_area_use_emissivity(self, index):
        '''
        Description:
          Read if area i is using custom emissivity
        Return:
          0=not used, 1=used
        '''
        cmd = '?AreaUseEmissivity('+str(index)+')\r\n'

        return int(self.get_data_by_cmd(cmd))


    def get_area_emissivity(self, index):
        '''
        Description:
          Read custom emissivity x of area i
        '''
        cmd = '?AreaEmissivity('+str(index)+')\r\n'

        return float(self.get_data_by_cmd(cmd))


    def get_area_distribution_mode_range(self, index):
        '''
        Description:
          Read the distribution range x1,x2 of area i
        '''
        cmd = '?AreaDistributionModeRange('+str(index)+')\r\n'

        return self.get_data_by_cmd(cmd)


    def get_range_count(self):
        '''
        Description:
          Read count of existing temperature ranges
        '''
        cmd = '?RangeCount\r\n'

        return int(self.get_data_by_cmd(cmd))


    def get_range_index(self):
        '''
        Description:
          Read the index of the current temperature range
        '''
        cmd = '?RangeIndex\r\n'

        return int(self.get_data_by_cmd(cmd))


    def get_range_min(self, index):
        '''
        Description:
          Read the low temperature of the temperature range with index i
        '''
        cmd = '?RangeMin('+str(index)+')\r\n'

        return self.convert_temp_data_to_value(self.get_data_by_cmd(cmd, 'ignore'))


    def get_range_max(self, index):
        '''
        Description:
          Read the high temperature of the temperature range with index i
        '''
        cmd = '?RangeMax('+str(index)+')\r\n'

        return self.convert_temp_data_to_value(self.get_data_by_cmd(cmd, 'ignore'))


    def get_fwver(self):
        '''
        Description:
          Read the firmware and hardware version of the imager
        '''
        cmd = '?FWVer\r\n'

        return self.get_data_by_cmd(cmd)


    def get_flag(self):
        '''
        Description:
          Read flag state
        Return:
          0 = flag is open, 1 = flag is closed
        '''
        cmd = '?Flag\r\n'

        return int(self.get_data_by_cmd(cmd))



    def get_focusmotor_min_pos(self):
        '''
        Description:
          Get minimum position of Xi's focus motor
        '''
        cmd = '?FocusmotorMinPos\r\n'

        return int(self.get_data_by_cmd(cmd))


    def get_focusmotor_max_pos(self):
        '''
        Description:
          Get maximum position of Xi's focus motor
        '''
        cmd = '?FocusmotorMaxPos\r\n'

        return int(self.get_data_by_cmd(cmd))


    def get_focusmotor_pos(self):
        '''
        Description:
          Get actual position of Xi's focus motor
        '''
        cmd = '?FocusmotorPos\r\n'

        return int(self.get_data_by_cmd(cmd))
