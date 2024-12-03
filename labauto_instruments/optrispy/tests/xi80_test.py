#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from optrispy.optris_interface import OptrisInterface


if __name__ ==  "__main__":
    xi80 = OptrisInterface()
    xi80.connect_optris()

    temp = xi80.get_temperature()
    print('Temp [degC]:', temp)

    temp_0 = xi80.get_temperature_index(0)
    print('Temp(0) [degC]:', temp_0)

    chip_temp = xi80.get_chip_temperature()
    print('Chip_temp [degC]:', chip_temp)

    flag_temp = xi80.get_flag_temperature()
    print('Flag_temp [degC]:', flag_temp)

    internal_temp = xi80.get_internal_temperature()
    print('Internal_temp [degC]:', internal_temp)

    emissivity = xi80.get_emissivity()
    print('Emissivity:', emissivity)

    transmissivity = xi80.get_transmissivity()
    print('Transmissivity:', transmissivity)

    ambient_temp = xi80.get_ambient_temperature()
    print('Ambient_temp [degC]:', ambient_temp)

    sn = xi80.get_serial_number()
    print('Serial number:', sn)

    measure_area_count = xi80.get_measure_area_count()
    print('Measure area count:', measure_area_count)

    area_mode_0 = xi80.get_area_mode(0)
    print('Area mode(0):', area_mode_0)

    area_use_emissivity_0 = xi80.get_area_use_emissivity(0)
    print('Area use emissivity(0):', area_use_emissivity_0)

    area_emissivity_0 = xi80.get_area_emissivity(0)
    print('Area emissivity(0):', area_emissivity_0)

    area_distribution_mode_range_0 = xi80.get_area_distribution_mode_range(0)
    print('Area distribution_mode_range(0):', area_distribution_mode_range_0)

    range_count = xi80.get_range_count()
    print('Range count:', range_count)

    range_index = xi80.get_range_index()
    print('Range index:', range_index)

    range_min_0 = xi80.get_range_min(0)
    print('Range min(0) [degC]:', range_min_0)

    range_max_0 = xi80.get_range_max(0)
    print('Range max(0) [degC]:', range_max_0)

    fwver = xi80.get_fwver()
    print('FWver:', fwver)

    flag = xi80.get_flag()
    print('Flag:', flag)

    focusmotor_min_pos = xi80.get_focusmotor_min_pos()
    print('FocusMotor min pos:', focusmotor_min_pos)

    focusmotor_max_pos = xi80.get_focusmotor_max_pos()
    print('FocusMotor max pos:', focusmotor_max_pos)

    focusmotor_pos = xi80.get_focusmotor_pos()
    print('FocusMotor pos:', focusmotor_pos)
