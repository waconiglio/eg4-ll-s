# -*- coding: utf-8 -*-

# Notes
# Added by https://github.com/tuxntoast

from battery import Battery, Cell

# from batters import Protection
from utils import logger, read_serial_data
from struct import unpack_from
from time import sleep
from pprint import pformat
import utils
import sys

#    Author: Pfitz /
#    Date: 01 Aug 2024
#    Version 2.0
#     - Tested / Updated to work with dbus-SerialBattery v1.3.20240705
#     - Added function get_max_temp() get_min_temp() - New calls by dbus-serial for adaptive charing
#     - Updates to get_balancing() function - Looks at default value from BMS Settings to indicate when
#       blancing mode would be active. Value are not pulled from the BMS config yet, and thus could be different
#     - Starting to add support for BMS channing / more then one battery unit
#        - Commands, and new function path
#        - Connect and Poll all BMS units in Communcation Chain
#        - Take in value from all BMS in chain, and summerize those to report to the Cerbo
#       Tasks:
#            - Passing all cell voltage to OS, but as different pack
#                Issue: DVCC will add all cell to find charge voltage, this should be sum of all cell in pack
#            - Command Check Sum Logic - Would allow for commands to be generated from code, rather then static
#    Version 1.1
#     Cell Voltage Implemented
#     Hardware Name / Version / Serial Implemented
#     Error / Warn / Protection Implemented
#     SoH / SoC State Implemented
#     Temp Implemented
#     Battery Voltage / Current

# Battery Tested on:
# Eg4 LL 12v 400 AH (single battery) x2
# One RS232 Cable to USB is needed when connecting to the Cerbo GX
# A networking cable should be connecte between each of the BMS RS485 Ports and the other BMS units
# The master unit or first unit should have a Dip Switch ID set to 16
# All other BMS should have a Dip switch setting of 1 - 15

class EG4_LL(Battery):
    def __init__(self, port, baud, address):

        super(EG4_LL, self).__init__(port, baud, address)
        self.cell_min_voltage = 0
        self.cell_max_voltage = None
        self.poll_interval = 5000
        self.type = self.BATTERYTYPE
        self.has_settings = 0
        self.reset_soc = 0
        self.soc_to_set = None
        self.runtime = 1  # TROUBLESHOOTING for no reply errors

    # Modbus uses 7C call vs Lifepower 7E, as return values do not correlate to the Lifepower ones if 7E is used.
    # at least on my own BMS.
    statuslogger = False
    debug = False  # Set to true for wordy debugging in logs
    debug_hex = False
    debug_config_hex = False
    debug_config = False
    batteryPackId = [ 16, 1 ]
    battery_stats = {}

    #balancing = 0
    BATTERYTYPE = "EG4 LL"
    balacing_text = "UNKNOWN"
    LENGTH_CHECK = 0
    LENGTH_CHECK
    LENGTH_POS = 2  # offset starting from 0
    LENGTH_FIXED = -1

    commands = {
      16 : {
        "HW" : b"\x10\x03\x00\x69\x00\x17\xD6\x99",
        "CELL" : b"\x10\x03\x00\x00\x00\x27\x06\x91"},
      1 : {
        "HW" : b"\x01\x03\x00\x69\x00\x17\xD5\xD8",
        "CELL" : b"\x01\x03\x00\x00\x00\x27\x05\xD0",
        "CONFIG" : b"\x01\x03\x00\x2D\x00\x5B\x94\x38"},
      2: {
        "HW" : b"\x02\x03\x00\x69\x00\x17\xD5\xEB",
        "CELL" : b"\x02\x03\x00\x00\x00\x27\x05\xE3"},
      3: {
        "HW" : b"\x03\x03\x00\x69\x00\x17\xD4\x3A",
        "CELL" : b"\x03\x03\x00\x00\x00\x27\x04\x32"},
      4: {
        "HW" : b"\x04\x03\x00\x69\x00\x17\xD5\x8D",
        "CELL" : b"\x04\x03\x00\x00\x00\x27\x05\x85"},
      5: {
        "HW" : b"\x05\x03\x00\x69\x00\x17\xD4\x5C",
        "CELL" : b"\x05\x03\x00\x00\x00\x27\x04\x54"},
      6: {
        "HW" : b"\x06\x03\x00\x69\x00\x17\xD4\x6F",
        "CELL" : b"\x06\x03\x00\x00\x00\x27\x04\x67"},
      7: {
        "HW" : b"\x07\x03\x00\x69\x00\x17\xD5\xBE",
        "CELL" : b"\x07\x03\x00\x00\x00\x27\x05\xB6"},
      8: {
        "HW" : b"\x08\x03\x00\x69\x00\x17\xD5\x41",
        "CELL" : b"\x08\x03\x00\x00\x00\x27\x05\x49"},
      9: {
        "HW" : b"\x09\x03\x00\x69\x00\x17\xD4\x90",
        "CELL" : b"\x09\x03\x00\x00\x00\x27\x04\x98"},
      10: {
        "HW" : b"\x0A\x03\x00\x69\x00\x17\xD4\xA3",
        "CELL" : b"\x0A\x03\x00\x00\x00\x27\x04\xAB"},
      11: {
        "HW" : b"\x0B\x03\x00\x69\x00\x17\xD5\x72",
        "CELL" : b"\x0B\x03\x00\x00\x00\x27\x05\x7A"},
      12: {
        "HW" : b"\x0C\x03\x00\x69\x00\x17\xD4\xC5",
        "CELL" : b"\x0C\x03\x00\x00\x00\x27\x04\xCD"},
      13: {
        "HW" : b"\x0D\x03\x00\x69\x00\x17\xD5\x14",
        "CELL" : b"\x0D\x03\x00\x00\x00\x27\x05\x1c"},
      14: {
        "HW" : b"\x0E\x03\x00\x69\x00\x17\xD5\x27",
        "CELL" : b"\x0E\x03\x00\x00\x00\x27\x05\x2F"},
      15: {
        "HW" : b"\x0F\x03\x00\x69\x00\x17\xD4\xF6",
        "CELL" : b"\x0F\x03\x00\x00\x00\x27\x04\xFE"}
    }

    def unique_identifier(self):
        return "4S12400190500001"

    def test_connection(self):
        # call a function that will connect to the battery, send a command and retrieve the result.
        # The result or call should be unique to this BMS. Battery name or version, etc.
        # Return True if success, False for failure
        try:
            self.battery_stats = {}
            BMS_list = self.discovery_pack()
            if len(BMS_list) > 0:
                if 16 in BMS_list:
                    self.battery_stats[16] = self.read_cell_details(16)
                    if self.battery_stats[16] is not False:
                        reply = self.rollupBatteryBank(self.battery_stats)
                        if reply != "Failed":
                            return True
                        else:
                            return False
                    else:
                        return False
        except Exception:
            (
                exception_type,
                exception_object,
                exception_traceback,
            ) = sys.exc_info()
            file = exception_traceback.tb_frame.f_code.co_filename
            line = exception_traceback.tb_lineno
            logger.error(
                f"Exception occurred: {repr(exception_object)} of type {exception_type} in {file} line #{line}"
            )

    def get_settings(self):
        # After successful  connection get_settings will be call to set up the battery.
        # Return True if success, False for failure
        id = 1
        battery_stats = {}
        for id in self.batteryPackId:
            hw_reply = self.read_hw_details(id)
            cell_reply = self.read_cell_details(id)
            if hw_reply is not False and cell_reply is not False:
                self.battery_stats[id] = { **cell_reply, **hw_reply }
            id+=1

        result = self.rollupBatteryBank(self.battery_stats)
        if (self.statuslogger is True) or (result == "Failed"):
            self.status_logger(self.battery_stats)
        return True

    def refresh_data(self):
        # call all functions that will refresh the battery data.
        # This will be called for every iteration (1 second)
        # Return True if success, False for failure
        result = self.read_battery_bank()
        if result is False:
            return False
        return True

    def read_gen_data(self):
        result = self.read_battery_bank()
        if result is False:
            return False
        return True

### ### ### ### ### ### ### ### ### ### ### ### ### ### ### ### ### ### ### ###

    def discovery_pack(self):
        bmsId = 1
        bmsChain = {}
        if not self.batteryPackId:
            while bmsId <= 16:
                attempts = 0
                while attempts < 3:
                    reply = self.read_serial_data_eg4_ll(self.commands[bmsId]["HW"])
                    if reply is not False:
                        bmsChain.update({bmsId : True})
                        break
                    attempts += 1
                    #sleep(3)
                bmsId += 1
        else:
            for Id in self.batteryPackId:
                attempts = 0
                while attempts < 1:
                    reply = self.read_serial_data_eg4_ll(self.commands[Id]["HW"])
                    if reply is not False:
                        bmsChain.update({Id : True})
                        break
                    attempts += 1
                    #sleep(3)
                bmsId += 1

        logger.info(f"Connected to BMS ID's: {pformat(bmsChain)}")
        return bmsChain

    def read_hw_details(self, id):
        battery = {}
        result = self.read_serial_data_eg4_ll(self.commands[id]["HW"])
        if result is False:
            return False

        battery.update({"hw_make" : result[2:25].decode("utf-8")})
        battery.update({"hw_version" : result[27:33].decode("utf-8")})
        battery.update({"hw_serial" : (result[33:48].decode("utf-8")+"_"+str(id))})
        self.serial_number = battery["hw_serial"]
        self.version = battery["hw_make"]
        self.hardware_version = battery["hw_version"]

        return battery

    def read_cell_details(self, id):

        battery = {}
        packet = self.read_serial_data_eg4_ll(self.commands[id]["CELL"])
        if packet is False:
            return False

        battery.update({"voltage" : int.from_bytes(packet[3:5], "big") / 100})
        battery.update({"current" : int.from_bytes(packet[5:7], "big", signed=True) / 100})
        battery.update({"capacity_remain" : int.from_bytes(packet[45:47], "big")})
        battery.update({"capacity" : int.from_bytes(packet[65:69], "big") / 3600 / 1000})
        battery.update({"max_battery_charge_current" : int.from_bytes(packet[47:49], "big")})
        battery.update({"soc" : int.from_bytes(packet[51:53], "big")})
        battery.update({"soh" : int.from_bytes(packet[49:51], "big")})
        battery.update({"cycles" : int.from_bytes(packet[61:65], "big")})
        battery.update({"temp1" : int.from_bytes(packet[39:41], "big", signed=True)})
        battery.update({"temp2" : int.from_bytes(packet[69:70], "big", signed=True)})
        battery.update({"temp_mos" : int.from_bytes(packet[70:71], "big", signed=True)})
        battery.update({"temp_max" : max(battery["temp1"], battery["temp2"])})
        battery.update({"temp_min" : min(battery["temp1"], battery["temp2"])})
        battery.update({"cell_count" : int.from_bytes(packet[75:77], "big")})
        battery.update({"status_hex" : packet[54:55].hex().upper()})
        battery.update({"warning_hex" : packet[55:57].hex().upper()})
        battery.update({"protection_hex" : packet[57:59].hex().upper()})
        battery.update({"error_hex" : packet[59:61].hex().upper()})
        battery.update({"heater_status" : packet[53:54].hex().upper()})
        startByte = 7
        endByte = 9
        cellId = 1
        cellVoltageList = []
        cellVoltageSum = 0
        while cellId <= battery["cell_count"]:
            cellNum = "cell"+str(cellId)
            cellVolt = int.from_bytes(packet[startByte:endByte], "big")/1000
            battery.update({cellNum : cellVolt})
            cellVoltageSum += float(battery[cellNum])
            cellVoltageList.append(battery.get(cellNum))
            startByte += 2
            endByte += 2
            cellId += 1
        battery.update({"cell_voltage" : cellVoltageSum})
        battery.update({"cell_max" : max(cellVoltageList)})
        battery.update({"cell_min" : min(cellVoltageList)})
        balancing_code = self.status_balancing(battery["cell_max"], battery["cell_min"], "code")
        balancing_text = self.status_balancing(battery["cell_max"], battery["cell_min"], "text")
        battery.update({"balancing_code" : balancing_code})
        battery.update({"balancing_text" : balancing_text})
        return battery

    def rollupBatteryBank(self, batteryBankStats):

        if self.battery_stats is False or self.battery_stats[16] is False:
            return "Failed"

        #logger.info(f"batteryBankStats: {pformat(batteryBankStats)}")
        self.voltage = self.battery_stats[16]["cell_voltage"]
        self.current = self.battery_stats[16]["current"]
        self.capacity_remain = self.battery_stats[16]["capacity_remain"]
        self.capacity = self.battery_stats[16]["capacity"]
        self.soc = self.battery_stats[16]["soc"]
        self.soh = self.battery_stats[16]["soh"]
        self.cycles = self.battery_stats[16]["cycles"]
        self.temp1 = self.battery_stats[16]["temp1"]
        self.temp2 = self.battery_stats[16]["temp2"]
        self.temp_mos = self.battery_stats[16]["temp_mos"]
        #self.serial_number = batteryBankStats[16]["hw_serial"]
        #self.version = batteryBankStats[16]["hw_make"]
        #self.hardware_version = batteryBankStats[16]["hw_version"]
        self.lookup_protection(self.battery_stats)
        self.lookup_warning(self.battery_stats)

        if len(self.battery_stats) > 1:
            for bmsId in self.battery_stats:
                if bmsId != 16:
                    if bmsId != False:
                        if self.battery_stats[bmsId] is not False:
                            self.voltage = (self.voltage + self.battery_stats[bmsId]["cell_voltage"]) / 2
                            self.current = round((self.current + self.battery_stats[bmsId]["current"]), 2)
                            self.capacity_remain = (self.capacity_remain + self.battery_stats[bmsId]["capacity_remain"])
                            self.capacity = (self.capacity + self.battery_stats[bmsId]["capacity"])
                            self.soc = (self.soc + self.battery_stats[bmsId]["soc"]) / 2
                            self.soh = (self.soh + self.battery_stats[bmsId]["soh"]) / 2
                            if self.battery_stats[bmsId]["cycles"] > self.cycles:
                                self.cycles = self.battery_stats[bmsId]["cycles"]
                            if self.battery_stats[bmsId]["temp1"] > self.temp1:
                                self.temp1 = self.battery_stats[bmsId]["temp1"]
                            if self.battery_stats[bmsId]["temp2"] > self.temp2:
                                self.temp2 = self.battery_stats[bmsId]["temp2"]
                            if self.battery_stats[bmsId]["temp_mos"] > self.temp_mos:
                                self.temp_mos = self.battery_stats[bmsId]["temp_mos"]

        self.temp_max = max(self.temp1, self.temp2)
        self.temp_min = min(self.temp1, self.temp2)
        self.cell_count = batteryBankStats[16]["cell_count"]
        self.min_battery_voltage = float(utils.MIN_CELL_VOLTAGE * self.cell_count)
        self.max_battery_voltage = float(utils.MAX_CELL_VOLTAGE * self.cell_count)

        if len(self.cells) != self.cell_count:
            self.cells = []
            for idx in range(self.cell_count):
                self.cells.append(Cell(False))

        self.cells[0].voltage = self.battery_stats[16]["cell1"]
        self.cells[1].voltage = self.battery_stats[16]["cell2"]
        self.cells[2].voltage = self.battery_stats[16]["cell3"]
        self.cells[3].voltage = self.battery_stats[16]["cell4"]
        #self.cells[4].voltage = batteryBankStats[1]["cell1"]
        #self.cells[5].voltage = batteryBankStats[1]["cell2"]
        #self.cells[6].voltage = batteryBankStats[1]["cell3"]
        #self.cells[7].voltage = batteryBankStats[1]["cell4"]


        self.cell_min_voltage = min(self.cells[0].voltage, self.cells[1].voltage,
                                    self.cells[2].voltage, self.cells[3].voltage)
                #                    self.cells[4].voltage, self.cells[5].voltage,
                #                    self.cells[6].voltage, self.cells[7].voltage)
        self.cell_max_voltage = max(self.cells[0].voltage, self.cells[1].voltage,
                                    self.cells[2].voltage, self.cells[3].voltage)
                #                    self.cells[4].voltage, self.cells[5].voltage,
                #                    self.cells[6].voltage, self.cells[7].voltage)
        return True

    def status_logger(self, batteryBankStats):
        if self.battery_stats is not False or self.battery_stats[16] is not False:
            logger.info("===== HW Info =====")
            logger.info(f"Battery Make/Model: {self.battery_stats[16]['hw_make']}")
            logger.info(f"Hardware Version: {self.battery_stats[16]['hw_version']}")
            for bmsId in self.battery_stats:
                if self.battery_stats[bmsId] is not False:
                    logger.info(f"Serial Number: {self.battery_stats[bmsId]['hw_serial']}")
            logger.info("===== Temp =====")
            logger.info(f"Temp 1: {self.temp1}c | Temp 2: {self.temp2}c | Temp Mos: {self.temp_mos}c")
            logger.info(f"Temp Max: {self.temp_max} | Temp Min: {self.temp_min}")
            for bmsId in self.battery_stats:
                if self.battery_stats[bmsId] is not False:
                    logger.info(f"Heater {bmsId} Status: {self.lookup_heater(self.battery_stats[bmsId]['heater_status'])}")
            logger.info("===== DVCC State =====")
            logger.info(f"DVCC Charger Mode: {self.charge_mode}")
            logger.info(f"DVCC Charge Voltage: {self.control_voltage}v")
            logger.info(
                f"Charge Current: {self.control_charge_current} | Discharge Current: {self.control_discharge_current}"
            )
            logger.info(
                f"Charge Limit: {self.charge_limitation} | Discharge Limit: {self.discharge_limitation}"
            )
            logger.info("===== BMS Data =====")
            logger.info("Voltage: "
                + "%.3fv" % self.voltage
                + " | Current: "
                + str(self.current)
                + "A"
            )
            logger.info(f"Capacity Left: {self.capacity_remain} of {self.capacity} AH")
            logger.info(f"SoC: {self.soc}%")
            logger.info(f"SoH: {self.soh}% | Cycle Count: {self.cycles}")
            logger.info(f"Balancing State: {self.balacing_text}")
            logger.info("===== Warning/Alarms =====")
            logger.info(f"  {self.lookup_warning(self.battery_stats)}")
            logger.info(f"  {self.lookup_protection(self.battery_stats)}")
            logger.info(f"  {self.lookup_error(self.battery_stats)}")
            logger.info("===== Pack Details =====")
            for bmsId in self.battery_stats:
                if self.battery_stats[bmsId] is not False:
                    cellId = 1
                    logger.info(f"  === BMS ID-{bmsId} ===")
                    logger.info(f"  State: {self.lookup_status(self.battery_stats[bmsId]['status_hex'])}")
                    logger.info(f"  Pack Balancing: {self.battery_stats[bmsId]['balancing_text']}")
                    logger.info(f"  Pack Voltage: {round((self.battery_stats[bmsId]['cell_voltage']),3)}v | Pack Current: {round((self.battery_stats[bmsId]['current']),2)}a")
                    logger.info("    = Cell Stats =")
                    while cellId <= self.battery_stats[bmsId]['cell_count']:
                        logger.info(f"  Cell {str(cellId)} Voltage: {self.battery_stats[bmsId]['cell'+str(cellId)]}")
                        cellId += 1
                    logger.info(f"  Cell Max/Min/Diff: ({self.battery_stats[bmsId]['cell_max']}/{self.battery_stats[bmsId]['cell_min']}/{round((self.battery_stats[bmsId]['cell_max'] - self.battery_stats[bmsId]['cell_min']), 3)})v")
        return True

    def lookup_warning(self, batteryBankStats):

        unique_codes = []
        for bmsId in batteryBankStats:
            if batteryBankStats[bmsId] is not False:
                if batteryBankStats[bmsId]["warning_hex"] not in unique_codes:
                    unique_codes.append(batteryBankStats[bmsId]["warning_hex"])
        warning_alarm = ""
        for code in unique_codes:
            if code == "0000":
                warning_alarm += "No Warnings - "+code
            elif code == "0001":
                warning_alarm += "Warning: "+code+" - Pack Over Voltage"
                self.voltage_high = 1
            elif code == "0002":
                warning_alarm += "Warning: "+code+" - Cell Over Voltage"
                self.voltage_cell_high = 1
            elif code == "0004":
                warning_alarm += "Warning: "+code+" - Pack Under Voltage"
                self.voltage_low = 1
            elif code == "0008":
                warning_alarm += "Warning: "+code+" - Cell Under Voltage"
                self.voltage_cell_low = 1
            elif code == "0010":
                warning_alarm += "Warning: "+code+" - Charge Over Current"
                self.current_over = 1
            elif code == "0020":
                warning_alarm += "Warning: "+code+" - Discharge Over Current"
                self.current_over = 1
            elif code == "0040":
                warning_alarm += "Warning: "+code+" - Ambient High Temp"
                self.temp_high_internal = 1
            elif code == "0080":
                warning_alarm += "Warning: "+code+" - Mosfets High Temp"
                self.temp_high_internal = 1
            elif code == "0100":
                warning_alarm += "Warning: "+code+" - Charge Over Temp"
                self.temp_high_charge = 1
            elif code == "0200":
                warning_alarm += "Warning: "+code+" - Discharge Over Temp"
                self.temp_high_discharge = 1
            elif code == "0400":
                warning_alarm += "Warning: "+code+" - Charge Under Temp"
                self.temp_low_charge = 1
            elif code == "1000":
                warning_alarm += "Warning: "+code+" - Low Capacity"
                self.soc_low = 1
            elif code == "2000":
                warning_alarm += "Warning: "+code+" - Float Stoped"
            elif code == "4000":
                warning_alarm += "Warning: "+code+" - UNKNOWN"
                self.internal_failure = 1
            else:
                warning_alarm += "Warning: "+code+" - UNKNOWN"

        return warning_alarm

    def lookup_protection(self, batteryBankStats):
        unique_codes = []
        for bmsId in batteryBankStats:
            if batteryBankStats[bmsId] is not False:
                if batteryBankStats[bmsId]["protection_hex"] not in unique_codes:
                    unique_codes.append(batteryBankStats[bmsId]["protection_hex"])
        protection_alarm = ""
        for code in unique_codes:
            if code == "0000":
                protection_alarm += "No Protection Events - "+code
            elif code == "0001":
                protection_alarm += "Protection: "+code+" - Pack Over Voltage"
                self.voltage_high = 2
            elif code == "0002":
                protection_alarm += "Protection: "+code+" - Cell Over Voltage"
                self.voltage_cell_high = 2
            elif code == "0004":
                protection_alarm += "Protection: "+code+" - Pack Under Voltage"
                self.voltage_low = 2
            elif code == "0008":
                protection_alarm += "Protection: "+code+" - Cell Under Voltage"
                self.voltage_cell_low = 2
            elif code == "0010":
                protection_alarm += "Protection: "+code+" - Charge Over Current"
                self.current_over = 2
            elif code == "0020":
                protection_alarm += "Protection: "+code+" - Discharge Over Current"
                self.current_over = 2
            elif code == "0040":
                protection_alarm += "Protection: "+code+" - High Ambient Temp"
                self.temp_high_internal = 2
            elif code == "0080":
                protection_alarm += "Protection: "+code+" - Mosfets High Temp"
                self.temp_high_internal = 2
            elif code == "0100":
                protection_alarm += "Protection: "+code+" - Charge Over Temp"
                self.temp_high_charge = 2
            elif code == "0200":
                protection_alarm += "Protection: "+code+" - Discharge Over Temp"
                self.temp_high_discharge = 2
            elif code == "0400":
                protection_alarm += "Protection: "+code+" - Charge Under Temp"
                self.temp_low_charge = 2
            elif code == "0800":
                protection_alarm += "Protection: "+code+" - Discharge Under Temp"
                self.temp_low_charge = 2
            elif code == "1000":
                protection_alarm += "Protection: "+code+" - Low Capacity"
                self.soc_low = 2
            elif code == "2000":
                protection_alarm += "Protection: "+code+" - Discharge SC"
            else:
                protection_alarm += "UNKNOWN: "+code
        return protection_alarm

    def lookup_error(self, batteryBankStats):
        unique_codes = []
        for bmsId in batteryBankStats:
            if batteryBankStats[bmsId] is not False:
                if batteryBankStats[bmsId]["error_hex"] not in unique_codes:
                    unique_codes.append(batteryBankStats[bmsId]["error_hex"])
        error_alarm = ""
        for code in unique_codes:
            if code == "0000":
                error_alarm = f"No Errors - "+code
            elif code == "0001":
                error_alarm = f"Error: "+code+" - Voltage Error"
            elif code == "0002":
                error_alarm = f"Error: "+code+" - Temperature Error"
            elif code == "0004":
                error_alarm = f"Error: "+code+" - Current Flow Error"
            elif code == "0010":
                error_alarm = f"Error: "+code+" - Cell Unbalanced"
            else:
                error_alarm = "UNKNOWN: "+code
        return error_alarm

    def lookup_status(self, status_hex):
        if status_hex == "00":
            status_code = "Standby"
        elif status_hex == "01":
            status_code = "Charging"
        elif status_hex == "02":
            status_code = "Discharging"
        elif status_hex == "04":
            status_code = "Protect"
        elif status_hex == "08":
            status_code = "Charging Limit"
        return status_code

    def lookup_heater(self, heater_status):
        if heater_status == "00":
            heater_state = False
        elif heater_status == "80":
            heater_state = True
        return heater_state

    def status_balancing(self, cell_max, cell_min, reply):
        balancer_current_delta = .40
        balancer_voltage = 3.40
        balacing_state = 0
        balacing_text = ""
        if (cell_max > balancer_voltage) and (round((cell_max - cell_min), 3) <= balancer_current_delta):
            balacing_state = 2
            balacing_text = "Finished"
        elif (cell_max - cell_min) >= balancer_current_delta:
            if cell_max >= balancer_voltage:
                balacing_state = 1
                balacing_text = "Balancing"
        else:
            balacing_state = 0
            balacing_text = "Off"

        if reply == "text":
            return balacing_text
        elif reply == "code":
            return balacing_state
        else:
            return False

    def read_battery_bank(self):
        id = 1
        for id in self.batteryPackId:
            dataPacket = self.read_cell_details(id)
            if dataPacket is not False: # if True
                self.battery_stats[id] = { **self.battery_stats[id], **dataPacket }
            sleep(.2)
            id+=1
        result = self.rollupBatteryBank(self.battery_stats)
        if self.statuslogger is True:
            self.status_logger(self.battery_stats)
        return True
        #return False

### ### ### ### ### ### ### ### ### ### ### ### ### ### ### ### ### ### ### ###

    def get_balancing(self):
        balancingSummery = []
        for bmsId in self.batteryPackId:
            if bmsId in self.battery_stats:
                if self.battery_stats[bmsId] is not False:
                    stateCode = self.status_balancing(self.battery_stats[bmsId]['cell_max'], self.battery_stats[bmsId]['cell_min'], "code")
                    balancingSummery.append(stateCode)
                else:
                    return 0

        balancingFinished = all(ele in balancingSummery for ele in [2])
        if balancingFinished is True:
            balacing_state = 2
            self.balacing_text = "Finished"
        elif 1 in balancingSummery:
            balacing_state = 1
            self.balacing_text = "Balancing"
        else:
            balacing_state = 0
            self.balacing_text = "Off"
        return balacing_state

    def get_max_temp(self):
        self.temp1 = self.battery_stats[16]["temp1"]
        self.temp2 = self.battery_stats[16]["temp2"]
        temp_max = max(self.temp1, self.temp2)
        return temp_max

    def get_min_temp(self):
        self.temp1 = self.battery_stats[16]["temp1"]
        self.temp2 = self.battery_stats[16]["temp2"]
        temp_min = min(self.temp1, self.temp2)
        return temp_min

    def read_bms_config(self):
        logger.info("Executed read_bms_config function... function needs to be written")
        return True

    def generate_command(self, command):
        # buffer = bytearray(self.command_address)
        # buffer += command
        return command

    def read_serial_data_eg4_ll(self, command):
        # use the read_serial_data() function to read the data and then do BMS specific checks (crc, start bytes, etc

        LENGTH_CHECK = 0
        LENGTH_CHECK
        LENGTH_POS = 2  # offset starting from 0
        LENGTH_FIXED = -1

        if self.debug:
            logger.info(f'Executed Command: {command.hex(":").upper()}')

        serial_data = read_serial_data(
            command, self.port, self.baud_rate, self.LENGTH_POS, self.LENGTH_CHECK
        )
        if not serial_data: #Test for False / No-Reply
            failedCommandHex = command.hex(":").upper()
            bmsId = int(failedCommandHex[0:2], 16)
            cmdId = failedCommandHex[9:11]
            if cmdId == "69":
                commandString = "Hardware"
            elif cmdId == "00":
                commandString = "Cell"
            elif cmdId == "2D":
                commandString = "Config"
            else:
                commandString = "UNKNOWN"
            logger.error(f'No Reply - BMS ID:{bmsId} Command-{commandString}')
            sleep(1)

            return False

        # Its not quite modbus, but psuedo modbus'ish'
        modbus_address, modbus_type, modbus_cmd, modbus_packet_length = unpack_from(
            "BBBB", serial_data
        )

        if self.debug:
            logger.info(f"Modbus Address: {modbus_address} [{hex(modbus_address)}]")
            logger.info(f"Modbus Type   : {modbus_type} [{hex(modbus_type)}]")
            logger.info(f"Modbus Command: {modbus_cmd} [{hex(modbus_cmd)}]")
            logger.info(
                f"Modbus PackLen: {modbus_packet_length} [{hex(modbus_packet_length)}]"
            )
            logger.info(f'Modbus Packet : [ {serial_data.hex(":").upper()} ]')
        return serial_data

