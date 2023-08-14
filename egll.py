# -*- coding: utf-8 -*-

# Undeprecate driver
from battery import Protection, Battery, Cell
from utils import *
from struct import *
import struct
import utils

#    Author: Pfitz /
#    Date: 6 Aug 2023
#    Version 1.0
#     Cell Voltage Implemented
#     Hardware Name / Version / Serial Implemented
#     Error / Warn / Protection Implemented
#     SoH / SoC State Implemented
#     Temp Implemented
#     Battery Voltage / Current

# Battery Tested on:
# Eg4 egll 12v 400 AH (single battery)

# BMS Documentation Sourced:
# https://eg4electronics.com/wp-content/uploads/2022/09/egll-MODBUS-Communication-Protocol_ENG-correct-1.pdf


# Work Items:
#  - Read BMS Config's
#  - Balancing

# add to file /data/etc/dbus-serialbattery/dbus-serialbattery.py
# from bms.lifepower import egll
# {"bms": egll, "baud": 9600, "address":b"\x7C"

class egll(Battery):
    def __init__(self, port, baud, address):

        super(egll, self).__init__(port, baud, address)
        self.type = self.BATTERYTYPE
        #self.port='/dev/ttyUSB0'
        self.command_address = b"\0x7C"  # 7C on my tianpower...

    # Modbus uses 7C call vs Lifepower 7E, as return values do not correlate to the Lifepower ones if 7E is used.
    # at least on my own BMS.
    debug = True  # Set to true for wordy debugging in logs
    balancing = 0
    BATTERYTYPE = "egll"
    LENGTH_CHECK = 0
    LENGTH_POS = 2  # offset starting from 0
    LENGTH_FIXED = -1

    command_get_version = b"\x01\x03\x00\x69\x00\x23\xD4\x0F" # Pulled from PC Client
    command_get_stats = b"\x01\x03\x00\x00\x00\x27\x05\xD0" # Pulled from PC Client
    command_get_config = b"\x01\x03\x00\x2D\x00\x5B\x94\x38" # Pulled from PC Client

    def test_connection(self):
        # call a function that will connect to the battery, send a command and retrieve the result.
        # The result or call should be unique to this BMS. Battery name or version, etc.
        # Return True if success, False for failure
        logger.info(f'egll test connection')
        result = False
        try:
            check1 = self.read_gen_data()
            sleep(.5)
            # get first data to show in startup log
            check2 = self.refresh_data()
            if check1 and check2:
                return True
        except Exception as err:
            logger.error(f"Unexpected {err=}, {type(err)=}")
            result = False

        return True

    def get_settings(self):
        # After successful  connection get_settings will be call to set up the battery.
        # Return True if success, False for failure

        self.max_battery_charge_current = utils.MAX_BATTERY_CHARGE_CURRENT
        self.max_battery_discharge_current = utils.MAX_BATTERY_DISCHARGE_CURRENT
        result = self.read_cell_data()
        self.poll_interval = 1000

        if result is False:
            return False

        return True

    def refresh_data(self):
        # call all functions that will refresh the battery data.
        # This will be called for every iteration (1 second)
        # Return True if success, False for failure
        result = self.read_cell_data()
        return result

    def read_gen_data(self):

        version = self.read_serial_data_egll(self.command_get_version)

        # check if connection success
        if version is False:
            return False
        else:
            return True

        #self.custom_field = version[2:27].decode("utf-8")
        #self.hardware_version = version[27:33].decode("utf-8")
        #self.unique_identifier = version[33:50].decode("utf-8")

    def read_cell_data(self):
        packet = self.read_serial_data_egll(self.command_get_stats)

        if packet is False:
            return False

        if (self.debug):
            logger.info(f'===== BMS Com Raw - Parsed =====')
            logger.info(f'Battery Voltage Raw: {packet[3:5].hex(":").upper()}')
            logger.info(f'Current RAW: {packet[5:7].hex(":").upper()}')
            logger.info(f'Capacity Remaining RAW: {packet[45:47].hex(":").upper()}')
            logger.info(f'Capacity RAW: {packet[65:69].hex(":").upper()}')
            logger.info(f'Cell Count RAW: {packet[75:77].hex(":").upper()}')
            logger.info(f'Max Charge Current RAW: {packet[47:49].hex(":").upper()}')
            logger.info(f'SoC RAW: {packet[51:53].hex(":").upper()}')
            logger.info(f'SoH Raw: {packet[49:51].hex(":").upper()}')
            logger.info(f'Cycles RAW: {packet[61:65].hex(":").upper()}')
            logger.info(f'======= TEMP RAW =======')
            logger.info(f'Temp Sensor Bits: {packet[69:77].hex(":").upper()}')
            logger.info(f'Temp 1 RAW: {packet[39:41].hex(":").upper()}')
            logger.info(f'Temp 2 RAW: {packet[69:70].hex(":").upper()}')
            logger.info(f'Temp 3 RAW: {packet[70:71].hex(":").upper()}')
            logger.info(f'Avg Temp Raw: {packet[41:43].hex(":").upper()}')
            logger.info(f'Temp Max Raw: {packet[43:45].hex(":").upper()}')

        self.voltage = int.from_bytes(packet[3:5], "big")/100
        self.current = int.from_bytes(packet[5:7], "big",signed=True)/100
        self.capacity_remain = int.from_bytes(packet[45:47], "big")
        self.capacity = (int.from_bytes(packet[65:69], "big")/3600/1000)
        self.max_battery_charge_current = int.from_bytes(packet[47:49], "big")
        self.soc = int.from_bytes(packet[51:53], "big")
        self.cycles = int.from_bytes(packet[61:65], "big")
        self.temp1 = int.from_bytes(packet[39:41], "big", signed=True)
        self.temp2 = int.from_bytes(packet[69:70], "big", signed=True)
        self.temp3 = int.from_bytes(packet[70:71], "big", signed=True)
        self.cell_count = int.from_bytes(packet[75:77], "big")
        self.min_battery_voltage = utils.MIN_CELL_VOLTAGE * self.cell_count
        self.max_battery_voltage = utils.MAX_CELL_VOLTAGE * self.cell_count

        cell_total = 0
        cell_start_pos = 7
        cell_end_pos = 9
        i = 0

        if len(self.cells) != self.cell_count:
            self.cells = []
            for idx in range(self.cell_count):
                self.cells.append(Cell(False))

        for c in range(self.cell_count):
            cell_voltage = int.from_bytes(packet[cell_start_pos:cell_end_pos], "big")/1000
            cell_total += cell_voltage
            cell_start_pos += 2
            cell_end_pos += 2
            self.cells[i].voltage = cell_voltage
            i += 1

        if packet[53:55].hex().upper() == "0000":
            status_code = (f'Status: {packet[53:55].hex().upper()} - Inactive/Stanby')
        elif packet[53:55].hex().upper() == "0001":
            status_code = (f'Status: {packet[53:55].hex().upper()} - Inactive/Charging')
        elif packet[53:55].hex().upper() == "0002":
            status_code = (f'Status: {packet[53:55].hex().upper()} - Inactive/Discharging')
        elif packet[53:55].hex().upper() == "0004":
            status_code = (f'Status: {packet[53:55].hex().upper()} - Inactive/Protect')
        elif packet[53:55].hex().upper() == "0008":
            status_code = (f'Status: {packet[53:55].hex().upper()} - Inactive/Charging Limit')
        elif packet[53:55].hex().upper() == "8000":
            status_code = (f'Status: {packet[53:55].hex().upper()} - Active/Stand By')
        elif packet[53:55].hex().upper() == "8001":
            status_code = (f'Status: {packet[53:55].hex().upper()} - Active/Charging')
        elif packet[53:55].hex().upper() == "8002":
            status_code = (f'Status: {packet[53:55].hex().upper()} - Active/Discharging')
        elif packet[53:55].hex().upper() == "8004":
            status_code = (f'Status: {packet[53:55].hex().upper()} - Active/Protect')
        elif packet[53:55].hex().upper() == "8008":
            status_code = (f'Status: {packet[53:55].hex().upper()} - Active/Charging Limit')

        if packet[55:57].hex().upper() == "0000":
            warning_alarm = (f'No Warnings - {packet[55:57].hex(":").upper()}')
        elif packet[55:57].hex().upper() == "0001":
            warning_alarm = (f'Warning: {packet[55:57].hex(":").upper()} - Pack Over Voltage')
            self.voltage_high = 1
        elif packet[55:57].hex().upper() == "0002":
            warning_alarm = (f'Warning: {packet[55:57].hex(":").upper()} - Cell Over Voltage')
            self.voltage_cell_high = 1
        elif packet[55:57].hex().upper() == "0004":
            warning_alarm = (f'Warning: {packet[55:57].hex(":").upper()} - Pack Under Voltage')
            self.voltage_low = 1
        elif packet[55:57].hex().upper() == "0008":
            warning_alarm = (f'Warning: {packet[55:57].hex(":").upper()} - Cell Under Voltage')
            self.voltage_cell_low = 1
        elif packet[55:57].hex().upper() == "0010":
            warning_alarm = (f'Warning: {packet[55:57].hex(":").upper()} - Charge Over Current')
            self.current_over = 1
        elif packet[55:57].hex().upper() == "0020":
            warning_alarm = (f'Warning: {packet[55:57].hex(":").upper()} - Discharge Over Current')
            self.current_over = 1
        elif packet[55:57].hex().upper() == "0040":
            warning_alarm = (f'Warning: {packet[55:57].hex(":").upper()} - Ambient High Temp')
            self.temp_high_internal = 1
        elif packet[55:57].hex().upper() == "0080":
            warning_alarm = (f'Warning: {packet[55:57].hex(":").upper()} - Mosfets High Temp')
            self.temp_high_internal = 1
        elif packet[55:57].hex().upper() == "0100":
            warning_alarm = (f'Warning: {packet[55:57].hex(":").upper()} - Charge Over Temp')
            self.temp_high_charge = 1
        elif packet[55:57].hex().upper() == "0200":
            warning_alarm = (f'Warning: {packet[55:57].hex(":").upper()} - Discharge Over Temp')
            self.temp_high_discharge = 1
        elif packet[55:57].hex().upper() == "0400":
            warning_alarm = (f'Warning: {packet[55:57].hex(":").upper()} - Charge Under Temp')
            self.temp_low_charge = 1
        elif packet[55:57].hex().upper() == "1000":
            warning_alarm = (f'Warning: {packet[55:57].hex(":").upper()} - Low Capacity')
            self.soc_low = 1
        elif packet[55:57].hex().upper() == "2000":
            warning_alarm = (f'Warning: {packet[55:57].hex(":").upper()} - Float Stoped')
        elif packet[55:57].hex().upper() == "4000":
            warning_alarm = (f'Warning: {packet[55:57].hex(":").upper()} - UNKNOWN')
            self.internal_failure = 1

        if packet[57:59].hex().upper() == "0000":
            protection_alarm = (f'No Protection Events - {packet[57:59].hex(":").upper()}')
        elif packet[57:59].hex().upper() == "0001":
            protection_alarm = (f'Protection: {packet[57:59].hex(":").upper()} - Pack Over Voltage')
            self.voltage_high = 2
        elif packet[57:59].hex().upper() == "0002":
            protection_alarm = (f'Protection: {packet[57:59].hex(":").upper()} - Cell Over Voltage')
            self.voltage_cell_high = 2
        elif packet[57:59].hex().upper() == "0004":
            protection_alarm = (f'Protection: {packet[57:59].hex(":").upper()} - Pack Under Voltage')
            self.voltage_low = 2
        elif packet[57:59].hex().upper() == "0008":
            protection_alarm = (f'Protection: {packet[57:59].hex(":").upper()} - Cell Under Voltage')
            self.voltage_cell_low = 2
        elif packet[57:59].hex().upper() == "0010":
            protection_alarm = (f'Protection: {packet[57:59].hex(":").upper()} - Charge Over Current')
            self.current_over = 2
        elif packet[57:59].hex().upper() == "0020":
            protection_alarm = (f'Protection: {packet[57:59].hex(":").upper()} - Discharge Over Current')
            self.current_over = 2
        elif packet[57:59].hex().upper() == "0040":
            protection_alarm = (f'Protection: {packet[57:59].hex(":").upper()} - High Ambient Temp')
            self.temp_high_internal = 2
        elif packet[57:59].hex().upper() == "0080":
            protection_alarm = (f'Protection: {packet[57:59].hex(":").upper()} - Mosfets High Temp')
            self.temp_high_internal = 2
        elif packet[57:59].hex().upper() == "0100":
            protection_alarm = (f'Protection: {packet[57:59].hex(":").upper()} - Charge Over Temp')
            self.temp_high_charge = 2
        elif packet[57:59].hex().upper() == "0200":
            protection_alarm = (f'Protection: {packet[57:59].hex(":").upper()} - Discharge Over Temp')
            self.temp_high_discharge = 2
        elif packet[57:59].hex().upper() == "0400":
            protection_alarm = (f'Protection: {packet[57:59].hex(":").upper()} - Charge Under Temp')
            self.temp_low_charge = 2
        elif packet[57:59].hex().upper() == "0800":
            protection_alarm = (f'Protection: {packet[57:59].hex(":").upper()} - Discharge Under Temp')
            self.temp_low_charge = 2
        elif packet[57:59].hex().upper() == "1000":
            protection_alarm = (f'Protection: {packet[57:59].hex(":").upper()} - Low Capacity')
            self.soc_low = 2
        elif packet[57:59].hex().upper() == "2000":
            protection_alarm = (f'Protection: {packet[57:59].hex(":").upper()} - Discharge SC')

        if packet[59:61].hex().upper() == "0000":
            error = (f'No Errors - {packet[59:61].hex(":").upper()}')
        elif packet[59:61].hex().upper() == "0001":
            error = (f'Error: {packet[59:61].hex(":").upper()} - Voltage Error')
        elif packet[59:61].hex().upper() == "0002":
            error = (f'Error: {packet[59:61].hex(":").upper()} - Temperature Error')
        elif packet[59:61].hex().upper() == "0004":
            error = (f'Error: {packet[59:61].hex(":").upper()} - Current Flow Error')
        elif packet[59:61].hex().upper() == "0010":
            error = (f'Error: {packet[59:61].hex(":").upper()} - Cell Unbalanced')

        logger.info(f'===== BMS Data =====')
        logger.info(f'Battery Make/Model: {str(self.custom_field)}')
        logger.info(f'Hardware Version: {str(self.hardware_version)}')
        #logger.info(f'Serial Number: {str(self.unique_identifier)}')
        logger.info(f'Voltage (BMS): {self.voltage}v')
        logger.info("Cell Total Voltage: " + "%.2fv" % cell_total)
        logger.info(f'Current: {self.current}A')
        logger.info(f' {status_code}')
        logger.info(f'SoC: {self.soc}%')
        logger.info(f'Capacity Left: {self.capacity_remain} of {self.capacity} AH')
        logger.info(f'===== Warning/Alarms =====')
        logger.info(f' {warning_alarm}')
        logger.info(f' {protection_alarm}')
        logger.info(f' {error}')
        logger.info(f'===== Temp =====')
        logger.info(f'Temp 1: {self.temp1} c - PCB')
        logger.info(f'Temp 2: {self.temp2} c')
        logger.info(f'Temp 3: {self.temp3} c')
        logger.info(f'Avg Temp: {int.from_bytes(packet[41:43], "big", signed=True)} c')
        logger.info(f'Temp Max: {int.from_bytes(packet[43:45], "big", signed=True)} c')
        logger.info(f'===== Battery Stats =====')
        logger.info(f'SoH: {int.from_bytes(packet[49:51], "big")}%')
        logger.info(f'Cycle Count: {self.cycles}')
        logger.info(f'Cell Count: {self.cell_count}')
        logger.info(f'Max Charging Current: {self.max_battery_charge_current} A')

        return True

    def read_temp_data(self):
        # Temp Data is collected when the cell data is read
        result = self.read_cell_data()
        if result is False:
            return False
        return True

    def get_balancing(self):
        return 1 if self.balancing or self.balancing == 2 else 0

    def read_bms_config(self):
        return True

    def generate_command(self, command):
        #buffer = bytearray(self.command_address)
        #buffer += command
        return command

    def read_serial_data_egll(self, command):
        # use the read_serial_data() function to read the data and then do BMS specific checks (crc, start bytes, etc

        if (self.debug):
            logger.info(f'Modbus CMD Address: {hex(self.command_address[0]).upper()}')
            runcommand = self.generate_command(command)
            logger.info(f'Executed Command: {runcommand.hex(":").upper()}')

        data = read_serial_data(
            self.generate_command(command),
            self.port,
            self.baud_rate,
            self.LENGTH_POS,
            self.LENGTH_CHECK
        )
        if (self.debug):
            #logger.info(f'Device Port: {self.port}')
            #logger.info(f'Baud Rate: {self.baud_rate}')
            #logger.info(f'Input Command: {command.hex(":").upper()}')
            logger.info(f'Return: {bytearray(data)}')

        if data is False:
            logger.error("read_serial_data_egll::Serial Data is Bad")
            return False

        # Its not quite modbus, but psuedo modbus'ish'
        modbus_address, modbus_type, modbus_cmd, modbus_packet_length = unpack_from(
        "BBBB", data
        )

        if hex(modbus_cmd) == '0x46':
            self.version = ( self.BATTERYTYPE + " ver ( " + str(data[0:29]), "utf-8" + ")" )
            self.custom_field = data[2:27].decode("utf-8")
            self.hardware_version = data[27:33].decode("utf-8")
            self.unique_identifier = data[33:50].decode("utf-8")
            logger.info(f'Serial Number: {str(unique_identifier)}')

        if (self.debug):
            logger.info(f'Modbus Address: {modbus_address} [{hex(modbus_address)}]')
            logger.info(f'Modbus Type   : {modbus_type} [{hex(modbus_type)}]')
            logger.info(f'Modbus Command: {modbus_cmd} [{hex(modbus_cmd)}]')
            logger.info(f'Modbus PackLen: {modbus_packet_length} [{hex(modbus_packet_length)}]')
            logger.info(f'Modbus Packet : [ {data.hex(":").upper()} ]')

        if modbus_type == 3:
            logger.info("== Modbus packet good ==")
            return data # Pass the full packet from the BMS
        else:
            logger.error(">>> ERROR: Incorrect Reply")
            return False
