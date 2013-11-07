/*
 * Copyright (c) 2011, Olivier MATZ <zer0@droids-corp.org>
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the University of California, Berkeley nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE REGENTS AND CONTRIBUTORS ``AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE REGENTS AND CONTRIBUTORS BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <aversive/pgmspace.h>

#include <string.h>
#include <stdlib.h>
#include <stdio.h>

#include "xbee_atcmd.h"

static const char PROGMEM atcmd0_name[] = "WR";
static const char PROGMEM atcmd0_desc[] = "write-param";
#ifdef CONFIG_MODULE_XBEE_ATCMD_HELP
static const char PROGMEM atcmd0_help[] =
	"Write parameter values to non-volatile memory.";
#endif

static const char PROGMEM atcmd1_name[] = "RE";
static const char PROGMEM atcmd1_desc[] = "restore-defaults";
#ifdef CONFIG_MODULE_XBEE_ATCMD_HELP
static const char PROGMEM atcmd1_help[] =
	"Restore module parameters to factory defaults.";
#endif

static const char PROGMEM atcmd2_name[] = "FR";
static const char PROGMEM atcmd2_desc[] = "soft-reset";
#ifdef CONFIG_MODULE_XBEE_ATCMD_HELP
static const char PROGMEM atcmd2_help[] =
	"Software Reset. Responds with 'OK' then performs a "
	"reset 100ms later.";
#endif

static const char PROGMEM atcmd3_name[] = "AC";
static const char PROGMEM atcmd3_desc[] = "apply-changes";
#ifdef CONFIG_MODULE_XBEE_ATCMD_HELP
static const char PROGMEM atcmd3_help[] =
	"Apply Changes without exiting command mode.";
#endif

static const char PROGMEM atcmd4_name[] = "R1";
static const char PROGMEM atcmd4_desc[] = "restore-compiled";
#ifdef CONFIG_MODULE_XBEE_ATCMD_HELP
static const char PROGMEM atcmd4_help[] =
	"Restore module parameters to compiled defaults.";
#endif

static const char PROGMEM atcmd5_name[] = "VL";
static const char PROGMEM atcmd5_desc[] = "version-long";
#ifdef CONFIG_MODULE_XBEE_ATCMD_HELP
static const char PROGMEM atcmd5_help[] =
	"Shows detailed version information including"
	"application build date and time.";
#endif

static const char PROGMEM atcmd6_name[] = "DH";
static const char PROGMEM atcmd6_desc[] = "dst-addr-high";
#ifdef CONFIG_MODULE_XBEE_ATCMD_HELP
static const char PROGMEM atcmd6_help[] =
	"Upper 32 bits of the 64-bit destination address (0 "
	"to 0xFFFFFFFF, default is 0x0000FFFF).";
#endif

static const char PROGMEM atcmd7_name[] = "DL";
static const char PROGMEM atcmd7_desc[] = "dst-addr-low";
#ifdef CONFIG_MODULE_XBEE_ATCMD_HELP
static const char PROGMEM atcmd7_help[] =
	"Lower 32 bits of the 64-bit destination address (0 "
	"to 0xFFFFFFFF, default is 0x0000FFFF).";
#endif

static const char PROGMEM atcmd8_name[] = "DD";
static const char PROGMEM atcmd8_desc[] = "device-type-id";
#ifdef CONFIG_MODULE_XBEE_ATCMD_HELP
static const char PROGMEM atcmd8_help[] =
	"Device Type Identifier, it can be used to differentiate "
	"multiple XBee-based products (0 to 0xFFFFFFFF, read-only, "
	"default is 0x40000).";
#endif

static const char PROGMEM atcmd9_name[] = "SH";
static const char PROGMEM atcmd9_desc[] = "src-addr-high";
#ifdef CONFIG_MODULE_XBEE_ATCMD_HELP
static const char PROGMEM atcmd9_help[] =
	"Upper 32 bits of the 64-bit source address (read-only).";
#endif

static const char PROGMEM atcmd10_name[] = "SL";
static const char PROGMEM atcmd10_desc[] = "src-addr-low";
#ifdef CONFIG_MODULE_XBEE_ATCMD_HELP
static const char PROGMEM atcmd10_help[] =
	"Lower 32 bits of the 64-bit source address (read-only).";
#endif

static const char PROGMEM atcmd11_name[] = "SE";
static const char PROGMEM atcmd11_desc[] = "src-endpoint";
#ifdef CONFIG_MODULE_XBEE_ATCMD_HELP
static const char PROGMEM atcmd11_help[] =
	"The application source endpoint for all data transmissions "
	"(0 to 0xFF, default is 0xE8).";
#endif

static const char PROGMEM atcmd12_name[] = "DE";
static const char PROGMEM atcmd12_desc[] = "dst-endpoint";
#ifdef CONFIG_MODULE_XBEE_ATCMD_HELP
static const char PROGMEM atcmd12_help[] =
	"The application destination endpoint for all data "
	"transmissions (0 to 0xFF, default is 0xE8).";
#endif

static const char PROGMEM atcmd13_name[] = "CI";
static const char PROGMEM atcmd13_desc[] = "cluster-id";
#ifdef CONFIG_MODULE_XBEE_ATCMD_HELP
static const char PROGMEM atcmd13_help[] =
	"Cluster Identifier for all data transmissions (0 to 0xFFFF, "
	"default is 0x11).";
#endif

static const char PROGMEM atcmd14_name[] = "NP";
static const char PROGMEM atcmd14_desc[] = "max-rf-payload";
#ifdef CONFIG_MODULE_XBEE_ATCMD_HELP
static const char PROGMEM atcmd14_help[] =
	"Maximum RF Payload Bytes that can be sent in a unicast "
	"transmission based on the current configuration (0 to "
	"0xFFFF).";
#endif

static const char PROGMEM atcmd15_name[] = "CE";
static const char PROGMEM atcmd15_desc[] = "coord-end-device";
#ifdef CONFIG_MODULE_XBEE_ATCMD_HELP
static const char PROGMEM atcmd15_help[] =
	"Coordinator/End Device, messaging mode of the module "
	"(0 - Normal, 1 - Indirect coordinator, 2 - Polling, default "
	"is 0).";
#endif

static const char PROGMEM atcmd16_name[] = "AP";
static const char PROGMEM atcmd16_desc[] = "api-mode";
#ifdef CONFIG_MODULE_XBEE_ATCMD_HELP
static const char PROGMEM atcmd16_help[] =
	"API mode (0 - off, 1 - on, 2 - on with escape sequences).";
#endif

static const char PROGMEM atcmd17_name[] = "AO";
static const char PROGMEM atcmd17_desc[] = "api-output-format";
#ifdef CONFIG_MODULE_XBEE_ATCMD_HELP
static const char PROGMEM atcmd17_help[] =
	"API Output Format (0 - standard [0x90 for RX], 1 - explicit "
	"addressing [0x91 for RX]).";
#endif

static const char PROGMEM atcmd18_name[] = "BD";
static const char PROGMEM atcmd18_desc[] = "baud-rate";
#ifdef CONFIG_MODULE_XBEE_ATCMD_HELP
static const char PROGMEM atcmd18_help[] =
	"Baud rate of serial interface (0-8 select preset standard "
	"rates, and 0x39 to 0x1c9c38 select baud rate).";
#endif

static const char PROGMEM atcmd19_name[] = "RO";
static const char PROGMEM atcmd19_desc[] = "packetization-timeout";
#ifdef CONFIG_MODULE_XBEE_ATCMD_HELP
static const char PROGMEM atcmd19_help[] =
	"Packetization Timeout: the inter-character silence required "
	"before packetization specified in character times (0 to 0xFF, "
	"default is 3).";
#endif

static const char PROGMEM atcmd20_name[] = "FT";
static const char PROGMEM atcmd20_desc[] = "flow-control-thres";
#ifdef CONFIG_MODULE_XBEE_ATCMD_HELP
static const char PROGMEM atcmd20_help[] =
	"Flow Control Threshhold. De-assert CTS and/or send XOFF when "
	"FT bytes are in the UART receive buffer. Re-assert CTS when "
	"less than FT - 16 bytes are in the UART receive buffer (0x11 "
	"to 0xEE, default is 0xBE).";
#endif

static const char PROGMEM atcmd21_name[] = "NB";
static const char PROGMEM atcmd21_desc[] = "parity";
#ifdef CONFIG_MODULE_XBEE_ATCMD_HELP
static const char PROGMEM atcmd21_help[] =
	"Parity (0 - no parity, 1 - even parity, 2 - odd parity, 3 - "
	"forced high parity, 4 - forced low parity). Default is 0.";
#endif

static const char PROGMEM atcmd22_name[] = "D7";
static const char PROGMEM atcmd22_desc[] = "dio7";
#ifdef CONFIG_MODULE_XBEE_ATCMD_HELP
static const char PROGMEM atcmd22_help[] =
	"DIO7 Configuration (0 - unmonitored input, 1 - CTS, 3 - "
	"digital input, 4 - digital output low, 5 - digital output "
	"high, 6 - RS-485 low Tx, 7 - RS-485 high Tx). Default is "
	"0.";
#endif

static const char PROGMEM atcmd23_name[] = "D6";
static const char PROGMEM atcmd23_desc[] = "dio6";
#ifdef CONFIG_MODULE_XBEE_ATCMD_HELP
static const char PROGMEM atcmd23_help[] =
	"DIO6 Configuration (0 - unmonitored input, 1 - RTS, 3 - "
	"digital input, 4 - digital output low, 5 - digital output "
	"high). Default is 0.";
#endif

static const char PROGMEM atcmd24_name[] = "P0";
static const char PROGMEM atcmd24_desc[] = "dio10-pwm0";
#ifdef CONFIG_MODULE_XBEE_ATCMD_HELP
static const char PROGMEM atcmd24_help[] =
	"DIO10/PWM0 Configuration. (0 - unmonitored input, 1 - RSSI, 2 "
	"- PWM0, 3 - digital input, 4 - digital output low, 5 - "
	"digital output high). Default is 1.";
#endif

static const char PROGMEM atcmd25_name[] = "P1";
static const char PROGMEM atcmd25_desc[] = "dio11-pwm1";
#ifdef CONFIG_MODULE_XBEE_ATCMD_HELP
static const char PROGMEM atcmd25_help[] =
	"DIO11/PWM1 Configuration. (0 - unmonitored input, 2 "
	"- PWM1, 3 - digital input, 4 - digital output low, 5 - "
	"digital output high). Default is 0.";
#endif

static const char PROGMEM atcmd26_name[] = "P2";
static const char PROGMEM atcmd26_desc[] = "dio12";
#ifdef CONFIG_MODULE_XBEE_ATCMD_HELP
static const char PROGMEM atcmd26_help[] =
	"DIO12 Configuration. (0 - unmonitored input, "
	"3 - digital input, 4 - digital output low, 5 - "
	"digital output high). Default is 0.";
#endif

static const char PROGMEM atcmd27_name[] = "RP";
static const char PROGMEM atcmd27_desc[] = "rssi-pwm";
#ifdef CONFIG_MODULE_XBEE_ATCMD_HELP
static const char PROGMEM atcmd27_help[] =
	"Time RSSI signal will be output after last transmission. "
	"When RP[] = 0xFF, output will always be on (0 - 0xFF, default "
	"is 0x28[] = 4 seconds).";
#endif

static const char PROGMEM atcmd28_name[] = "1S";
static const char PROGMEM atcmd28_desc[] = "sensor-sample";
#ifdef CONFIG_MODULE_XBEE_ATCMD_HELP
static const char PROGMEM atcmd28_help[] =
	"Forces a sample to be taken on an XBee Sensor device.";
#endif

static const char PROGMEM atcmd29_name[] = "D0";
static const char PROGMEM atcmd29_desc[] = "dio0-ad0";
#ifdef CONFIG_MODULE_XBEE_ATCMD_HELP
static const char PROGMEM atcmd29_help[] =
	"AD0/DIO0 Configuration. (0 - unmonitored input, 1 - "
	"commission button enabled, 2 - analog input, 3 - digital "
	"input, 4 - digital output low, 5 - digital output high). "
	"Default is 1.";
#endif

static const char PROGMEM atcmd30_name[] = "D1";
static const char PROGMEM atcmd30_desc[] = "dio1-ad1";
#ifdef CONFIG_MODULE_XBEE_ATCMD_HELP
static const char PROGMEM atcmd30_help[] =
	"AD1/DIO1 Configuration. (0 - unmonitored input, "
	"2 - analog input, 3 - digital input, 4 - digital output "
	"low, 5 - digital output high). Default is 0.";
#endif

static const char PROGMEM atcmd31_name[] = "D2";
static const char PROGMEM atcmd31_desc[] = "dio2-ad2";
#ifdef CONFIG_MODULE_XBEE_ATCMD_HELP
static const char PROGMEM atcmd31_help[] =
	"AD2/DIO2 Configuration. (0 - unmonitored input, "
	"2 - analog input, 3 - digital input, 4 - digital output "
	"low, 5 - digital output high). Default is 0.";
#endif

static const char PROGMEM atcmd32_name[] = "D3";
static const char PROGMEM atcmd32_desc[] = "dio3-ad3";
#ifdef CONFIG_MODULE_XBEE_ATCMD_HELP
static const char PROGMEM atcmd32_help[] =
	"AD3/DIO3 Configuration. (0 - unmonitored input, "
	"2 - analog input, 3 - digital input, 4 - digital output "
	"low, 5 - digital output high). Default is 0.";
#endif

static const char PROGMEM atcmd33_name[] = "D4";
static const char PROGMEM atcmd33_desc[] = "dio4-ad4";
#ifdef CONFIG_MODULE_XBEE_ATCMD_HELP
static const char PROGMEM atcmd33_help[] =
	"AD4/DIO4 Configuration. (0 - unmonitored input, "
	"2 - analog input, 3 - digital input, 4 - digital output "
	"low, 5 - digital output high). Default is 0.";
#endif

static const char PROGMEM atcmd34_name[] = "D5";
static const char PROGMEM atcmd34_desc[] = "dio5-ad5";
#ifdef CONFIG_MODULE_XBEE_ATCMD_HELP
static const char PROGMEM atcmd34_help[] =
	"AD4/DIO4 Configuration. (0 - unmonitored input, 1 - LED, "
	"2 - analog input, 3 - digital input, 4 - digital output "
	"low, 5 - digital output high). Default is 1.";
#endif

static const char PROGMEM atcmd35_name[] = "D8";
static const char PROGMEM atcmd35_desc[] = "dio8-sleep-rq";
#ifdef CONFIG_MODULE_XBEE_ATCMD_HELP
static const char PROGMEM atcmd35_help[] =
	"DIO8/SLEEP_RQ Configuration. (0 - unmonitored input, 1 - LED, "
	"2 - analog input, 3 - digital input, 4 - digital output "
	"low, 5 - digital output high). Default is 0. When used as "
	"SLEEP_RQ, the D8 parameter should be configured in mode 0 "
	"or 3.";
#endif

static const char PROGMEM atcmd36_name[] = "D9";
static const char PROGMEM atcmd36_desc[] = "dio9-on-sleep";
#ifdef CONFIG_MODULE_XBEE_ATCMD_HELP
static const char PROGMEM atcmd36_help[] =
	"DIO9/ON_SLEEP Configuration. (0 - unmonitored input, 1 - "
	"ON/SLEEP, 2 - analog input, 3 - digital input, 4 - digital "
	"output low, 5 - digital output high). Default is ?.";
#endif

static const char PROGMEM atcmd37_name[] = "PR";
static const char PROGMEM atcmd37_desc[] = "pull-up-resistor";
#ifdef CONFIG_MODULE_XBEE_ATCMD_HELP
static const char PROGMEM atcmd37_help[] =
	"Pull-up Resistor. Bit field that configures the internal "
	"pull-up resistors for the I/O lines (bit set = pull-up "
	"enabled). Range is from 0 to 0x1FFF, default is 0x1FFF.";
#endif

static const char PROGMEM atcmd38_name[] = "M0";
static const char PROGMEM atcmd38_desc[] = "pwm0-out-level";
#ifdef CONFIG_MODULE_XBEE_ATCMD_HELP
static const char PROGMEM atcmd38_help[] =
	"PWM0 Output Level. The line should be configured as a PWM "
	"output using the P0 command (0 to 0x3FF, default is 0).";
#endif

static const char PROGMEM atcmd39_name[] = "M1";
static const char PROGMEM atcmd39_desc[] = "pwm1-out-level";
#ifdef CONFIG_MODULE_XBEE_ATCMD_HELP
static const char PROGMEM atcmd39_help[] =
	"PWM1 Output Level. The line should be configured as a PWM "
	"output using the P1 command (0 to 0x3FF, default is 0).";
#endif

static const char PROGMEM atcmd40_name[] = "LT";
static const char PROGMEM atcmd40_desc[] = "led-blink-time";
#ifdef CONFIG_MODULE_XBEE_ATCMD_HELP
static const char PROGMEM atcmd40_help[] =
	"Associate LED Blink Time (should be enabled through D5 ";
#endif

static const char PROGMEM atcmd41_name[] = "IS";
static const char PROGMEM atcmd41_desc[] = "force-sample";
#ifdef CONFIG_MODULE_XBEE_ATCMD_HELP
static const char PROGMEM atcmd41_help[] =
	"Forces a read of all enabled digital and "
	"analog input lines.";
#endif

static const char PROGMEM atcmd42_name[] = "IC";
static const char PROGMEM atcmd42_desc[] = "digital-change-detect";
#ifdef CONFIG_MODULE_XBEE_ATCMD_HELP
static const char PROGMEM atcmd42_help[] =
	"I/O Digital Change Detection. If a pin is enabled as a "
	"digital input/output, the IC command can be used to "
	"force an immediate I/O sample transmission when the DIO "
	"state changes. IC is a bitmask, range is 0 to 0xFFFF, "
	"default is 0";
#endif

static const char PROGMEM atcmd43_name[] = "IR";
static const char PROGMEM atcmd43_desc[] = "io-sample-rate";
#ifdef CONFIG_MODULE_XBEE_ATCMD_HELP
static const char PROGMEM atcmd43_help[] =
	"IO Sample Rate for periodic sampling. If zero, periodic "
	"sampling is disabled. Else the value is in milliseconds "
	"(range 0 to 0xFFFF, default is 0).";
#endif

static const char PROGMEM atcmd44_name[] = "CB";
static const char PROGMEM atcmd44_desc[] = "comissioning-button";
#ifdef CONFIG_MODULE_XBEE_ATCMD_HELP
static const char PROGMEM atcmd44_help[] =
	"Commissioning Pushbutton, simulate commissioning button "
	"in software. The parameter value should be set to the number "
	"of button presses to be simulated (range is 0 to 4).";
#endif

static const char PROGMEM atcmd45_name[] = "VR";
static const char PROGMEM atcmd45_desc[] = "firmware-version";
#ifdef CONFIG_MODULE_XBEE_ATCMD_HELP
static const char PROGMEM atcmd45_help[] =
	"Firmware version of the module (read only).";
#endif

static const char PROGMEM atcmd46_name[] = "HV";
static const char PROGMEM atcmd46_desc[] = "hardware-version";
#ifdef CONFIG_MODULE_XBEE_ATCMD_HELP
static const char PROGMEM atcmd46_help[] =
	"Hardware version of the module (read only).";
#endif

static const char PROGMEM atcmd47_name[] = "CK";
static const char PROGMEM atcmd47_desc[] = "config-code";
#ifdef CONFIG_MODULE_XBEE_ATCMD_HELP
static const char PROGMEM atcmd47_help[] =
	"Configuration Code, that can be used as a quick "
	"check to determine if a node has been configured as "
	"desired (read-only, 0-0xFFFFFFFF).";
#endif

static const char PROGMEM atcmd48_name[] = "ER";
static const char PROGMEM atcmd48_desc[] = "rf-errors";
#ifdef CONFIG_MODULE_XBEE_ATCMD_HELP
static const char PROGMEM atcmd48_help[] =
	"Number of times a packet was received which contained errors "
	"of some sort. Read-only, saturate at 0xFFFF.";
#endif

static const char PROGMEM atcmd49_name[] = "GD";
static const char PROGMEM atcmd49_desc[] = "good-packets";
#ifdef CONFIG_MODULE_XBEE_ATCMD_HELP
static const char PROGMEM atcmd49_help[] =
	"Number of good received frames. Read-only, saturate at "
	"0xFFFF.";
#endif

static const char PROGMEM atcmd50_name[] = "RP";
static const char PROGMEM atcmd50_desc[] = "rssi-pwm-timer";
#ifdef CONFIG_MODULE_XBEE_ATCMD_HELP
static const char PROGMEM atcmd50_help[] =
	"RSSI PWM timer, the time in tenth of seconds that the RSSI "
	"output indicating signal strength will remain active after "
	"the last reception (1 to 0xff, default is 0x20 = 3.2 secs).";
#endif

static const char PROGMEM atcmd51_name[] = "TR";
static const char PROGMEM atcmd51_desc[] = "tx-errors";
#ifdef CONFIG_MODULE_XBEE_ATCMD_HELP
static const char PROGMEM atcmd51_help[] =
	"Transmission Errors, the number of MAC frames that "
	"exhaust MAC retries without ever receiving a MAC "
	"acknowledgement message. Read-only, saturate at 0xFFFF.";
#endif

static const char PROGMEM atcmd52_name[] = "TP";
static const char PROGMEM atcmd52_desc[] = "temperature";
#ifdef CONFIG_MODULE_XBEE_ATCMD_HELP
static const char PROGMEM atcmd52_help[] =
	"Temperature. Read module temperature in (tenths of ?) "
	"Celsius. Negatives temperatures can be returned (read-only, "
	"from 0xff74 [-140] to 0x0258 [600]).";
#endif

static const char PROGMEM atcmd53_name[] = "DB";
static const char PROGMEM atcmd53_desc[] = "rx-signal-strength";
#ifdef CONFIG_MODULE_XBEE_ATCMD_HELP
static const char PROGMEM atcmd53_help[] =
	"Received Signal Strength of the last received RF data "
	"packet measured in -dBm. For example if DB returns 0x60, "
	"then the RSSI of the last packet received was -96dBm "
	"(read-only).";
#endif

static const char PROGMEM atcmd54_name[] = "DC";
static const char PROGMEM atcmd54_desc[] = "duty-cycle";
#ifdef CONFIG_MODULE_XBEE_ATCMD_HELP
static const char PROGMEM atcmd54_help[] =
	"Duty Cycle. Returns a current usage percentage of the "
	"10% duty cycle measured over the period of 1 hour "
	"(read-only, from 0 to 0x64).";
#endif

static const char PROGMEM atcmd55_name[] = "RC";
static const char PROGMEM atcmd55_desc[] = "rssi-for-channel";
#ifdef CONFIG_MODULE_XBEE_ATCMD_HELP
static const char PROGMEM atcmd55_help[] =
	"Reads the dBm level (RSSI) of the designated "
	"channel.";
#endif

static const char PROGMEM atcmd56_name[] = "R#";
static const char PROGMEM atcmd56_desc[] = "reset-number";
#ifdef CONFIG_MODULE_XBEE_ATCMD_HELP
static const char PROGMEM atcmd56_help[] =
	"Tells the reason for the last module reset (0 - Power up "
	"reset, 2 - Watchdog reset, 3 - Software reset, 4 - Reset "
	"line reset, 5 - Brownout reset). Read-only.";
#endif

static const char PROGMEM atcmd57_name[] = "TA";
static const char PROGMEM atcmd57_desc[] = "tx-ack-errors";
#ifdef CONFIG_MODULE_XBEE_ATCMD_HELP
static const char PROGMEM atcmd57_help[] =
	"Transmit Acknowlegement Errors. Incremented once for "
	"each failed ack retry (read-only, from 0 to 0xFFFF).";
#endif

static const char PROGMEM atcmd58_name[] = "%V";
static const char PROGMEM atcmd58_desc[] = "supply-voltage";
#ifdef CONFIG_MODULE_XBEE_ATCMD_HELP
static const char PROGMEM atcmd58_help[] =
	"Voltage on the Vcc pin in mV (read-only, from 0 to 0xF00).";
#endif

static const char PROGMEM atcmd59_name[] = "CT";
static const char PROGMEM atcmd59_desc[] = "cmd-mode-timeout";
#ifdef CONFIG_MODULE_XBEE_ATCMD_HELP
static const char PROGMEM atcmd59_help[] =
	"Command Mode Timeout: the period of inactivity (no valid "
	"commands received) after which the RF module automatically "
	"exits AT Command Mode and returns to Idle Mode (2 to 0x1770, "
	"default is 0x64).";
#endif

static const char PROGMEM atcmd60_name[] = "CN";
static const char PROGMEM atcmd60_desc[] = "exit-cmd-mode";
#ifdef CONFIG_MODULE_XBEE_ATCMD_HELP
static const char PROGMEM atcmd60_help[] =
	"Exit Command Mode.";
#endif

static const char PROGMEM atcmd61_name[] = "GT";
static const char PROGMEM atcmd61_desc[] = "guard-times";
#ifdef CONFIG_MODULE_XBEE_ATCMD_HELP
static const char PROGMEM atcmd61_help[] =
	"Guard Times: period of silence in ms before and after the "
	"Command Sequence Characters of the AT Command Mode Sequence, "
	"used to prevent inadvertent entrance into AT Command Mode "
	"(0 to 0xFFFF, default is 0x3E8).";
#endif

static const char PROGMEM atcmd62_name[] = "CC";
static const char PROGMEM atcmd62_desc[] = "command-chars";
#ifdef CONFIG_MODULE_XBEE_ATCMD_HELP
static const char PROGMEM atcmd62_help[] =
	"Command Character used between guard times of the AT Command "
	"Mode Sequence (0 to 0xFF, default is 0x2B).";
#endif

static const char PROGMEM atcmd63_name[] = "ID";
static const char PROGMEM atcmd63_desc[] = "network-id";
#ifdef CONFIG_MODULE_XBEE_ATCMD_HELP
static const char PROGMEM atcmd63_help[] =
	"Network ID. Nodes must have the same network identifier "
	"to communicate (0 to 0x7FFF, default is 0x7FFF).";
#endif

static const char PROGMEM atcmd64_name[] = "NT";
static const char PROGMEM atcmd64_desc[] = "ndisc-timeout";
#ifdef CONFIG_MODULE_XBEE_ATCMD_HELP
static const char PROGMEM atcmd64_help[] =
	"Node Discover Timeout, time in tenth of secs a node will "
	"spend discovering other nodes when ND or DN is issued (0 "
	"to 0xFC, default is 0x82).";
#endif

static const char PROGMEM atcmd65_name[] = "NI";
static const char PROGMEM atcmd65_desc[] = "node-id";
#ifdef CONFIG_MODULE_XBEE_ATCMD_HELP
static const char PROGMEM atcmd65_help[] =
	"Node Identifier in printable ASCII characters. This string is "
	"returned as part of the ATND (Network Discover) command. This "
	"identifier is also used with the ATDN (Destination Node) "
	"command. The string contains up to 20 byte ASCII string, "
	"default is a space character.";
#endif

static const char PROGMEM atcmd66_name[] = "DN";
static const char PROGMEM atcmd66_desc[] = "disc-node";
#ifdef CONFIG_MODULE_XBEE_ATCMD_HELP
static const char PROGMEM atcmd66_help[] = /* XXX */
	"Resolves a Node Identifier string to a physical address "
	"(case sensitive). 0xFFFE and the 64bits extended address are "
	"returned.";
#endif

static const char PROGMEM atcmd67_name[] = "ND";
static const char PROGMEM atcmd67_desc[] = "network-discover";
#ifdef CONFIG_MODULE_XBEE_ATCMD_HELP
static const char PROGMEM atcmd67_help[] = "Network Discovery, see doc"; /* XXX */
#endif

static const char PROGMEM atcmd68_name[] = "NO";
static const char PROGMEM atcmd68_desc[] = "ndisc-options";
#ifdef CONFIG_MODULE_XBEE_ATCMD_HELP
static const char PROGMEM atcmd68_help[] =
	"Network Discovery Options, a bitfield value that changes the "
	"behavior of the ND command (bit0 - Append DD value, bit1 - "
	"Local device sends ND response frame when ND is issued). "
	"Default is 0.";
#endif

static const char PROGMEM atcmd69_name[] = "EE";
static const char PROGMEM atcmd69_desc[] = "security enable";
#ifdef CONFIG_MODULE_XBEE_ATCMD_HELP
static const char PROGMEM atcmd69_help[] =
	"Enable or disable 128-bit AES encryption (0 or 1, 0 is the "
	"default).";
#endif

static const char PROGMEM atcmd70_name[] = "KY"; /* XXX */;
static const char PROGMEM atcmd70_desc[] = "security-key";
#ifdef CONFIG_MODULE_XBEE_ATCMD_HELP
static const char PROGMEM atcmd70_help[] =
	"The 128bits security key (the command is write-only).";
#endif

static const char PROGMEM atcmd71_name[] = "MT";
static const char PROGMEM atcmd71_desc[] = "bcast-multi-xmit";
#ifdef CONFIG_MODULE_XBEE_ATCMD_HELP
static const char PROGMEM atcmd71_help[] =
	"Number of additional MAC-level broadcast transmissions. All "
	"broadcast packets are transmitted MT+1 times to ensure "
	"it is received (0 to 0xF, default is 3).";
#endif

static const char PROGMEM atcmd72_name[] = "RR";
static const char PROGMEM atcmd72_desc[] = "unicast-retries";
#ifdef CONFIG_MODULE_XBEE_ATCMD_HELP
static const char PROGMEM atcmd72_help[] =
	"Number of additional MAC-level packet delivery attempts for "
	"unicast transactions. If RR is non-zero, packets sent from "
	"the radio will request an acknowledgement, and can be resent "
	"up to RR times if no acknowledgement is received. (0 to 0xF, "
	"default is 10).";
#endif

static const char PROGMEM atcmd73_name[] = "PL";
static const char PROGMEM atcmd73_desc[] = "power-level";
#ifdef CONFIG_MODULE_XBEE_ATCMD_HELP
static const char PROGMEM atcmd73_help[] =
	"Power Level of RF transmitter (0 - 1mW, 1 - 23mW, 2 - 100mW, "
	"3 - 158 mW, 4 - 316 mW). Default is 4.";
#endif

static const char PROGMEM atcmd74_name[] = "SM";
static const char PROGMEM atcmd74_desc[] = "sleep-mode";
#ifdef CONFIG_MODULE_XBEE_ATCMD_HELP
static const char PROGMEM atcmd74_help[] =
	"Sleep Mode (0 - disabled, 1 - pin sleep, 4 - async cyclic "
	"sleep, 5 - async cyclic sleep with pin wakeup). Default "
	"is 0.";
#endif

static const char PROGMEM atcmd75_name[] = "SO";
static const char PROGMEM atcmd75_desc[] = "sleep-options";
#ifdef CONFIG_MODULE_XBEE_ATCMD_HELP
static const char PROGMEM atcmd75_help[] =
	"Sleep Options bitmask (bit8 - always wake for ST time). "
	"Default is 0.";
#endif

static const char PROGMEM atcmd76_name[] = "ST";
static const char PROGMEM atcmd76_desc[] = "wake-time";
#ifdef CONFIG_MODULE_XBEE_ATCMD_HELP
static const char PROGMEM atcmd76_help[] =
	"Wake Time: the amount of time in ms that the module will stay "
	"awake after receiving RF or serial data (from 0x45 to "
	"0x36EE80, default is 0x7D0 = 2 secs).";
#endif

static const char PROGMEM atcmd77_name[] = "SP";
static const char PROGMEM atcmd77_desc[] = "sleep-period";
#ifdef CONFIG_MODULE_XBEE_ATCMD_HELP
static const char PROGMEM atcmd77_help[] =
	"Sleep Period: the amount of time in 10ms unit the module will "
	"sleep per cycle. For a node operating as an Indirect "
	"Messaging Coordinator, this command defines the amount of "
	"time that it will hold an indirect message for an end device. "
	"The coordinator will hold the message for (2.5 * SP). Range "
	"is from 1 to 1440000, default is 200 (2 secs).";
#endif

static const char PROGMEM atcmd78_name[] = "SN";
static const char PROGMEM atcmd78_desc[] = "num-sleep-periods";
#ifdef CONFIG_MODULE_XBEE_ATCMD_HELP
static const char PROGMEM atcmd78_help[] =
	"Number of Sleep Periods that must elapse between assertions "
	"of the ON_SLEEP line during the wake time of asynchronous "
	"cyclic sleep (1 to 0xFFFF, default is 1).";
#endif

static const char PROGMEM atcmd79_name[] = "WH";
static const char PROGMEM atcmd79_desc[] = "wake-host";
#ifdef CONFIG_MODULE_XBEE_ATCMD_HELP
static const char PROGMEM atcmd79_help[] = "Wake Host time. If it is set to a non-zero value, it "
	"specifies the time in ms that the device should allow after "
	"waking from sleep before sending data out the UART or "
	"transmitting an I/O sample. If serial characters are "
	"received, the WH timer is stopped immediately. Range is "
	"from 0 to 0xFFFF, default is 0.";
#endif

const struct xbee_atcmd PROGMEM xbee_atcmd_list[] = {
	{
		/* "WR" */
		atcmd0_name,
		atcmd0_desc,
		XBEE_ATCMD_F_PARAM_NONE | XBEE_ATCMD_F_WRITE,
#ifdef CONFIG_MODULE_XBEE_ATCMD_HELP
		atcmd0_help,
#endif
	},
	{
		/* "RE" */
		atcmd1_name,
		atcmd1_desc,
		XBEE_ATCMD_F_PARAM_NONE | XBEE_ATCMD_F_WRITE,
#ifdef CONFIG_MODULE_XBEE_ATCMD_HELP
		atcmd1_help,
#endif
	},
	{
		/* "FR" */
		atcmd2_name,
		atcmd2_desc,
		XBEE_ATCMD_F_PARAM_NONE | XBEE_ATCMD_F_WRITE,
#ifdef CONFIG_MODULE_XBEE_ATCMD_HELP
		atcmd2_help,
#endif
	},
	{
		/* "AC" */
		atcmd3_name,
		atcmd3_desc,
		XBEE_ATCMD_F_PARAM_NONE | XBEE_ATCMD_F_WRITE,
#ifdef CONFIG_MODULE_XBEE_ATCMD_HELP
		atcmd3_help,
#endif
	},
	{
		/* "R1" */
		atcmd4_name,
		atcmd4_desc,
		XBEE_ATCMD_F_PARAM_NONE | XBEE_ATCMD_F_WRITE,
#ifdef CONFIG_MODULE_XBEE_ATCMD_HELP
		atcmd4_help,
#endif
	},
	{
		/* "VL" */
		atcmd5_name,
		atcmd5_desc,
		XBEE_ATCMD_F_PARAM_NONE | XBEE_ATCMD_F_WRITE,
#ifdef CONFIG_MODULE_XBEE_ATCMD_HELP
		atcmd5_help,
#endif
	},
	{
		/* "DH" */
		atcmd6_name,
		atcmd6_desc,
		XBEE_ATCMD_F_PARAM_U32 | XBEE_ATCMD_F_READ | XBEE_ATCMD_F_WRITE,
#ifdef CONFIG_MODULE_XBEE_ATCMD_HELP
		atcmd6_help,
#endif
	},
	{
		/* "DL" */
		atcmd7_name,
		atcmd7_desc,
		XBEE_ATCMD_F_PARAM_U32 | XBEE_ATCMD_F_READ | XBEE_ATCMD_F_WRITE,
#ifdef CONFIG_MODULE_XBEE_ATCMD_HELP
		atcmd7_help,
#endif
	},
	{
		/* "DD" */
		atcmd8_name,
		atcmd8_desc,
		XBEE_ATCMD_F_PARAM_U32 | XBEE_ATCMD_F_READ,
#ifdef CONFIG_MODULE_XBEE_ATCMD_HELP
		atcmd8_help,
#endif
	},
	{
		/* "SH" */
		atcmd9_name,
		atcmd9_desc,
		XBEE_ATCMD_F_PARAM_U32 | XBEE_ATCMD_F_READ,
#ifdef CONFIG_MODULE_XBEE_ATCMD_HELP
		atcmd9_help,
#endif
	},
	{
		/* "SL" */
		atcmd10_name,
		atcmd10_desc,
		XBEE_ATCMD_F_PARAM_U32 | XBEE_ATCMD_F_READ,
#ifdef CONFIG_MODULE_XBEE_ATCMD_HELP
		atcmd10_help,
#endif
	},
	{
		/* "SE" */
		atcmd11_name,
		atcmd11_desc,
		XBEE_ATCMD_F_PARAM_U8 | XBEE_ATCMD_F_READ | XBEE_ATCMD_F_WRITE,
#ifdef CONFIG_MODULE_XBEE_ATCMD_HELP
		atcmd11_help,
#endif
	},
	{
		/* "DE" */
		atcmd12_name,
		atcmd12_desc,
		XBEE_ATCMD_F_PARAM_U8 | XBEE_ATCMD_F_READ | XBEE_ATCMD_F_WRITE,
#ifdef CONFIG_MODULE_XBEE_ATCMD_HELP
		atcmd12_help,
#endif
	},
	{
		/* "CI" */
		atcmd13_name,
		atcmd13_desc,
		XBEE_ATCMD_F_PARAM_U16 | XBEE_ATCMD_F_READ | XBEE_ATCMD_F_WRITE,
#ifdef CONFIG_MODULE_XBEE_ATCMD_HELP
		atcmd13_help,
#endif
	},
	{
		/* "NP" */
		atcmd14_name,
		atcmd14_desc,
		XBEE_ATCMD_F_PARAM_U16 | XBEE_ATCMD_F_READ,
#ifdef CONFIG_MODULE_XBEE_ATCMD_HELP
		atcmd14_help,
#endif
	},
	{
		/* "CE" */
		atcmd15_name,
		atcmd15_desc,
		XBEE_ATCMD_F_PARAM_U8 | XBEE_ATCMD_F_READ | XBEE_ATCMD_F_WRITE,
#ifdef CONFIG_MODULE_XBEE_ATCMD_HELP
		atcmd15_help,
#endif
	},
	{
		/* "AP" */
		atcmd16_name,
		atcmd16_desc,
		XBEE_ATCMD_F_PARAM_U8 | XBEE_ATCMD_F_READ | XBEE_ATCMD_F_WRITE,
#ifdef CONFIG_MODULE_XBEE_ATCMD_HELP
		atcmd16_help,
#endif
	},
	{
		/* "AO" */
		atcmd17_name,
		atcmd17_desc,
		XBEE_ATCMD_F_PARAM_U8 | XBEE_ATCMD_F_READ | XBEE_ATCMD_F_WRITE,
#ifdef CONFIG_MODULE_XBEE_ATCMD_HELP
		atcmd17_help,
#endif
	},
	{
		/* "BD" */
		atcmd18_name,
		atcmd18_desc,
		XBEE_ATCMD_F_PARAM_U32 | XBEE_ATCMD_F_READ | XBEE_ATCMD_F_WRITE,
#ifdef CONFIG_MODULE_XBEE_ATCMD_HELP
		atcmd18_help,
#endif
	},
	{
		/* "RO" */
		atcmd19_name,
		atcmd19_desc,
		XBEE_ATCMD_F_PARAM_U8 | XBEE_ATCMD_F_READ | XBEE_ATCMD_F_WRITE,
#ifdef CONFIG_MODULE_XBEE_ATCMD_HELP
		atcmd19_help,
#endif
	},
	{
		/* "FT" */
		atcmd20_name,
		atcmd20_desc,
		XBEE_ATCMD_F_PARAM_U8 | XBEE_ATCMD_F_READ | XBEE_ATCMD_F_WRITE,
#ifdef CONFIG_MODULE_XBEE_ATCMD_HELP
		atcmd20_help,
#endif
	},
	{
		/* "NB" */
		atcmd21_name,
		atcmd21_desc,
		XBEE_ATCMD_F_PARAM_U8 | XBEE_ATCMD_F_READ | XBEE_ATCMD_F_WRITE,
#ifdef CONFIG_MODULE_XBEE_ATCMD_HELP
		atcmd21_help,
#endif
	},
	{
		/* "D7" */
		atcmd22_name,
		atcmd22_desc,
		XBEE_ATCMD_F_PARAM_U8 | XBEE_ATCMD_F_READ | XBEE_ATCMD_F_WRITE,
#ifdef CONFIG_MODULE_XBEE_ATCMD_HELP
		atcmd22_help,
#endif
	},
	{
		/* "D6" */
		atcmd23_name,
		atcmd23_desc,
		XBEE_ATCMD_F_PARAM_U8 | XBEE_ATCMD_F_READ | XBEE_ATCMD_F_WRITE,
#ifdef CONFIG_MODULE_XBEE_ATCMD_HELP
		atcmd23_help,
#endif
	},
	{
		/* "P0" */
		atcmd24_name,
		atcmd24_desc,
		XBEE_ATCMD_F_PARAM_U8 | XBEE_ATCMD_F_READ | XBEE_ATCMD_F_WRITE,
#ifdef CONFIG_MODULE_XBEE_ATCMD_HELP
		atcmd24_help,
#endif
	},
	{
		/* "P1" */
		atcmd25_name,
		atcmd25_desc,
		XBEE_ATCMD_F_PARAM_U8 | XBEE_ATCMD_F_READ | XBEE_ATCMD_F_WRITE,
#ifdef CONFIG_MODULE_XBEE_ATCMD_HELP
		atcmd25_help,
#endif
	},
	{
		/* "P2" */
		atcmd26_name,
		atcmd26_desc,
		XBEE_ATCMD_F_PARAM_U8 | XBEE_ATCMD_F_READ | XBEE_ATCMD_F_WRITE,
#ifdef CONFIG_MODULE_XBEE_ATCMD_HELP
		atcmd26_help,
#endif
	},
	{
		/* "RP" */
		atcmd27_name,
		atcmd27_desc,
		XBEE_ATCMD_F_PARAM_U8 | XBEE_ATCMD_F_READ | XBEE_ATCMD_F_WRITE,
#ifdef CONFIG_MODULE_XBEE_ATCMD_HELP
		atcmd27_help,
#endif
	},
	{
		/* "1S" */
		atcmd28_name,
		atcmd28_desc,
		XBEE_ATCMD_F_PARAM_NONE | XBEE_ATCMD_F_WRITE,
#ifdef CONFIG_MODULE_XBEE_ATCMD_HELP
		atcmd28_help,
#endif
	},
	{
		/* "D0" */
		atcmd29_name,
		atcmd29_desc,
		XBEE_ATCMD_F_PARAM_U8 | XBEE_ATCMD_F_READ | XBEE_ATCMD_F_WRITE,
#ifdef CONFIG_MODULE_XBEE_ATCMD_HELP
		atcmd29_help,
#endif
	},
	{
		/* "D1" */
		atcmd30_name,
		atcmd30_desc,
		XBEE_ATCMD_F_PARAM_U8 | XBEE_ATCMD_F_READ | XBEE_ATCMD_F_WRITE,
#ifdef CONFIG_MODULE_XBEE_ATCMD_HELP
		atcmd30_help,
#endif
	},
	{
		/* "D2" */
		atcmd31_name,
		atcmd31_desc,
		XBEE_ATCMD_F_PARAM_U8 | XBEE_ATCMD_F_READ | XBEE_ATCMD_F_WRITE,
#ifdef CONFIG_MODULE_XBEE_ATCMD_HELP
		atcmd31_help,
#endif
	},
	{
		/* "D3" */
		atcmd32_name,
		atcmd32_desc,
		XBEE_ATCMD_F_PARAM_U8 | XBEE_ATCMD_F_READ | XBEE_ATCMD_F_WRITE,
#ifdef CONFIG_MODULE_XBEE_ATCMD_HELP
		atcmd32_help,
#endif
	},
	{
		/* "D4" */
		atcmd33_name,
		atcmd33_desc,
		XBEE_ATCMD_F_PARAM_U8 | XBEE_ATCMD_F_READ | XBEE_ATCMD_F_WRITE,
#ifdef CONFIG_MODULE_XBEE_ATCMD_HELP
		atcmd33_help,
#endif
	},
	{
		/* "D5" */
		atcmd34_name,
		atcmd34_desc,
		XBEE_ATCMD_F_PARAM_U8 | XBEE_ATCMD_F_READ | XBEE_ATCMD_F_WRITE,
#ifdef CONFIG_MODULE_XBEE_ATCMD_HELP
		atcmd34_help,
#endif
	},
	{
		/* "D8" */
		atcmd35_name,
		atcmd35_desc,
		XBEE_ATCMD_F_PARAM_U8 | XBEE_ATCMD_F_READ | XBEE_ATCMD_F_WRITE,
#ifdef CONFIG_MODULE_XBEE_ATCMD_HELP
		atcmd35_help,
#endif
	},
	{
		/* "D9" */
		atcmd36_name,
		atcmd36_desc,
		XBEE_ATCMD_F_PARAM_U8 | XBEE_ATCMD_F_READ | XBEE_ATCMD_F_WRITE,
#ifdef CONFIG_MODULE_XBEE_ATCMD_HELP
		atcmd36_help,
#endif
	},
	{
		/* "PR" */
		atcmd37_name,
		atcmd37_desc,
		XBEE_ATCMD_F_PARAM_U8 | XBEE_ATCMD_F_READ | XBEE_ATCMD_F_WRITE,
#ifdef CONFIG_MODULE_XBEE_ATCMD_HELP
		atcmd37_help,
#endif
	},
	{
		/* "M0" */
		atcmd38_name,
		atcmd38_desc,
		XBEE_ATCMD_F_PARAM_U16 | XBEE_ATCMD_F_READ | XBEE_ATCMD_F_WRITE,
#ifdef CONFIG_MODULE_XBEE_ATCMD_HELP
		atcmd38_help,
#endif
	},
	{
		/* "M1" */
		atcmd39_name,
		atcmd39_desc,
		XBEE_ATCMD_F_PARAM_U16 | XBEE_ATCMD_F_READ | XBEE_ATCMD_F_WRITE,
#ifdef CONFIG_MODULE_XBEE_ATCMD_HELP
		atcmd39_help,
#endif
	},
	{
		/* "LT" */
		atcmd40_name,
		atcmd40_desc,
		XBEE_ATCMD_F_PARAM_U8 | XBEE_ATCMD_F_READ | XBEE_ATCMD_F_WRITE,
#ifdef CONFIG_MODULE_XBEE_ATCMD_HELP
		atcmd40_help,
#endif
	},
	{
		/* "IS" */
		atcmd41_name,
		atcmd41_desc,
		XBEE_ATCMD_F_PARAM_NONE | XBEE_ATCMD_F_WRITE,
#ifdef CONFIG_MODULE_XBEE_ATCMD_HELP
		atcmd41_help,
#endif
	},
	{
		/* "IC" */
		atcmd42_name,
		atcmd42_desc,
		XBEE_ATCMD_F_PARAM_U16 | XBEE_ATCMD_F_READ | XBEE_ATCMD_F_WRITE,
#ifdef CONFIG_MODULE_XBEE_ATCMD_HELP
		atcmd42_help,
#endif
	},
	{
		/* "IR" */
		atcmd43_name,
		atcmd43_desc,
		XBEE_ATCMD_F_PARAM_U16 | XBEE_ATCMD_F_READ | XBEE_ATCMD_F_WRITE,
#ifdef CONFIG_MODULE_XBEE_ATCMD_HELP
		atcmd43_help,
#endif
	},
	{
		/* "CB" */
		atcmd44_name,
		atcmd44_desc,
		XBEE_ATCMD_F_PARAM_U8 | XBEE_ATCMD_F_READ | XBEE_ATCMD_F_WRITE,
#ifdef CONFIG_MODULE_XBEE_ATCMD_HELP
		atcmd44_help,
#endif
	},
	{
		/* "VR" */
		atcmd45_name,
		atcmd45_desc,
		XBEE_ATCMD_F_PARAM_U32 | XBEE_ATCMD_F_READ,
#ifdef CONFIG_MODULE_XBEE_ATCMD_HELP
		atcmd45_help,
#endif
	},
	{
		/* "HV" */
		atcmd46_name,
		atcmd46_desc,
		XBEE_ATCMD_F_PARAM_U16 | XBEE_ATCMD_F_READ,
#ifdef CONFIG_MODULE_XBEE_ATCMD_HELP
		atcmd46_help,
#endif
	},
	{
		/* "CK" */
		atcmd47_name,
		atcmd47_desc,
		XBEE_ATCMD_F_PARAM_U32 | XBEE_ATCMD_F_READ,
#ifdef CONFIG_MODULE_XBEE_ATCMD_HELP
		atcmd47_help,
#endif
	},
	{
		/* "ER" */
		atcmd48_name,
		atcmd48_desc,
		XBEE_ATCMD_F_PARAM_U16 | XBEE_ATCMD_F_READ,
#ifdef CONFIG_MODULE_XBEE_ATCMD_HELP
		atcmd48_help,
#endif
	},
	{
		/* "GD" */
		atcmd49_name,
		atcmd49_desc,
		XBEE_ATCMD_F_PARAM_U16 | XBEE_ATCMD_F_READ,
#ifdef CONFIG_MODULE_XBEE_ATCMD_HELP
		atcmd49_help,
#endif
	},
	{
		/* "RP" */
		atcmd50_name,
		atcmd50_desc,
		XBEE_ATCMD_F_PARAM_U8 | XBEE_ATCMD_F_READ | XBEE_ATCMD_F_WRITE,
#ifdef CONFIG_MODULE_XBEE_ATCMD_HELP
		atcmd50_help,
#endif
	},
	{
		/* "TR" */
		atcmd51_name,
		atcmd51_desc,
		XBEE_ATCMD_F_PARAM_U16 | XBEE_ATCMD_F_READ,
#ifdef CONFIG_MODULE_XBEE_ATCMD_HELP
		atcmd51_help,
#endif
	},
	{
		/* "TP" */
		atcmd52_name,
		atcmd52_desc,
		XBEE_ATCMD_F_PARAM_S16 | XBEE_ATCMD_F_READ,
#ifdef CONFIG_MODULE_XBEE_ATCMD_HELP
		atcmd52_help,
#endif
	},
	{
		/* "DB" */
		atcmd53_name,
		atcmd53_desc,
		XBEE_ATCMD_F_PARAM_U16 | XBEE_ATCMD_F_READ,
#ifdef CONFIG_MODULE_XBEE_ATCMD_HELP
		atcmd53_help,
#endif
	},
	{
		/* "DC" */
		atcmd54_name,
		atcmd54_desc,
		XBEE_ATCMD_F_PARAM_U8 | XBEE_ATCMD_F_READ,
#ifdef CONFIG_MODULE_XBEE_ATCMD_HELP
		atcmd54_help,
#endif
	},
	{
		/* "RC" */
		atcmd55_name,
		atcmd55_desc,
		XBEE_ATCMD_F_PARAM_U8 | XBEE_ATCMD_F_READ,
#ifdef CONFIG_MODULE_XBEE_ATCMD_HELP
		atcmd55_help,
#endif
	},
	{
		/* "R#" */
		atcmd56_name,
		atcmd56_desc,
		XBEE_ATCMD_F_PARAM_U8 | XBEE_ATCMD_F_READ,
#ifdef CONFIG_MODULE_XBEE_ATCMD_HELP
		atcmd56_help,
#endif
	},
	{
		/* "TA" */
		atcmd57_name,
		atcmd57_desc,
		XBEE_ATCMD_F_PARAM_U16 | XBEE_ATCMD_F_READ,
#ifdef CONFIG_MODULE_XBEE_ATCMD_HELP
		atcmd57_help,
#endif
	},
	{
		/* "%V" */
		atcmd58_name,
		atcmd58_desc,
		XBEE_ATCMD_F_PARAM_U8 | XBEE_ATCMD_F_READ,
#ifdef CONFIG_MODULE_XBEE_ATCMD_HELP
		atcmd58_help,
#endif
	},
	{
		/* "CT" */
		atcmd59_name,
		atcmd59_desc,
		XBEE_ATCMD_F_PARAM_U16 | XBEE_ATCMD_F_READ | XBEE_ATCMD_F_WRITE,
#ifdef CONFIG_MODULE_XBEE_ATCMD_HELP
		atcmd59_help,
#endif
	},
	{
		/* "CN" */
		atcmd60_name,
		atcmd60_desc,
		XBEE_ATCMD_F_PARAM_NONE | XBEE_ATCMD_F_WRITE,
#ifdef CONFIG_MODULE_XBEE_ATCMD_HELP
		atcmd60_help,
#endif
	},
	{
		/* "GT" */
		atcmd61_name,
		atcmd61_desc,
		XBEE_ATCMD_F_PARAM_U16 | XBEE_ATCMD_F_READ | XBEE_ATCMD_F_WRITE,
#ifdef CONFIG_MODULE_XBEE_ATCMD_HELP
		atcmd61_help,
#endif
	},
	{
		/* "CC" */
		atcmd62_name,
		atcmd62_desc,
		XBEE_ATCMD_F_PARAM_U8 | XBEE_ATCMD_F_READ | XBEE_ATCMD_F_WRITE,
#ifdef CONFIG_MODULE_XBEE_ATCMD_HELP
		atcmd62_help,
#endif
	},
	{
		/* "ID" */
		atcmd63_name,
		atcmd63_desc,
		XBEE_ATCMD_F_PARAM_U16 | XBEE_ATCMD_F_READ | XBEE_ATCMD_F_WRITE,
#ifdef CONFIG_MODULE_XBEE_ATCMD_HELP
		atcmd63_help,
#endif
	},
	{
		/* "NT" */
		atcmd64_name,
		atcmd64_desc,
		XBEE_ATCMD_F_PARAM_U8 | XBEE_ATCMD_F_READ | XBEE_ATCMD_F_WRITE,
#ifdef CONFIG_MODULE_XBEE_ATCMD_HELP
		atcmd64_help,
#endif
	},
	{
		/* "NI" */
		atcmd65_name,
		atcmd65_desc,
		XBEE_ATCMD_F_PARAM_STRING_20B | XBEE_ATCMD_F_READ |
		XBEE_ATCMD_F_WRITE,
#ifdef CONFIG_MODULE_XBEE_ATCMD_HELP
		atcmd65_help,
#endif
	},
	{
		/* "DN" */
		atcmd66_name,
		atcmd66_desc,
		XBEE_ATCMD_F_PARAM_STRING_20B | XBEE_ATCMD_F_READ |
		XBEE_ATCMD_F_WRITE,
#ifdef CONFIG_MODULE_XBEE_ATCMD_HELP
		atcmd66_help,
#endif
	},
	{
		/* "ND" */
		atcmd67_name,
		atcmd67_desc,
		XBEE_ATCMD_F_PARAM_NONE | XBEE_ATCMD_F_WRITE,
#ifdef CONFIG_MODULE_XBEE_ATCMD_HELP
		atcmd67_help,
#endif
	},
	{
		/* "NO" */
		atcmd68_name,
		atcmd68_desc,
		XBEE_ATCMD_F_PARAM_U8 | XBEE_ATCMD_F_READ | XBEE_ATCMD_F_WRITE,
#ifdef CONFIG_MODULE_XBEE_ATCMD_HELP
		atcmd68_help,
#endif
	},
	{
		/* "EE" */
		atcmd69_name,
		atcmd69_desc,
		XBEE_ATCMD_F_PARAM_U8 | XBEE_ATCMD_F_READ | XBEE_ATCMD_F_WRITE,
#ifdef CONFIG_MODULE_XBEE_ATCMD_HELP
		atcmd69_help,
#endif
	},
	{
		/* "KY"  XXX */
		atcmd70_name,
		atcmd70_desc,
		XBEE_ATCMD_F_PARAM_HEXBUF_16B | XBEE_ATCMD_F_WRITE,
#ifdef CONFIG_MODULE_XBEE_ATCMD_HELP
		atcmd70_help,
#endif
	},
	{
		/* "MT" */
		atcmd71_name,
		atcmd71_desc,
		XBEE_ATCMD_F_PARAM_U8 | XBEE_ATCMD_F_READ | XBEE_ATCMD_F_WRITE,
#ifdef CONFIG_MODULE_XBEE_ATCMD_HELP
		atcmd71_help,
#endif
	},
	{
		/* "RR" */
		atcmd72_name,
		atcmd72_desc,
		XBEE_ATCMD_F_PARAM_U8 | XBEE_ATCMD_F_READ | XBEE_ATCMD_F_WRITE,
#ifdef CONFIG_MODULE_XBEE_ATCMD_HELP
		atcmd72_help,
#endif
	},
	{
		/* "PL" */
		atcmd73_name,
		atcmd73_desc,
		XBEE_ATCMD_F_PARAM_U8 | XBEE_ATCMD_F_READ | XBEE_ATCMD_F_WRITE,
#ifdef CONFIG_MODULE_XBEE_ATCMD_HELP
		atcmd73_help,
#endif
	},
	{
		/* "SM" */
		atcmd74_name,
		atcmd74_desc,
		XBEE_ATCMD_F_PARAM_U8 | XBEE_ATCMD_F_READ | XBEE_ATCMD_F_WRITE,
#ifdef CONFIG_MODULE_XBEE_ATCMD_HELP
		atcmd74_help,
#endif
	},
	{
		/* "SO" */
		atcmd75_name,
		atcmd75_desc,
		XBEE_ATCMD_F_PARAM_U8 | XBEE_ATCMD_F_READ | XBEE_ATCMD_F_WRITE,
#ifdef CONFIG_MODULE_XBEE_ATCMD_HELP
		atcmd75_help,
#endif
	},
	{
		/* "ST" */
		atcmd76_name,
		atcmd76_desc,
		XBEE_ATCMD_F_PARAM_U32 | XBEE_ATCMD_F_READ | XBEE_ATCMD_F_WRITE,
#ifdef CONFIG_MODULE_XBEE_ATCMD_HELP
		atcmd76_help,
#endif
	},
	{
		/* "SP" */
		atcmd77_name,
		atcmd77_desc,
		XBEE_ATCMD_F_PARAM_U32 | XBEE_ATCMD_F_READ | XBEE_ATCMD_F_WRITE,
#ifdef CONFIG_MODULE_XBEE_ATCMD_HELP
		atcmd77_help,
#endif
	},
	{
		/* "SN" */
		atcmd78_name,
		atcmd78_desc,
		XBEE_ATCMD_F_PARAM_U16 | XBEE_ATCMD_F_READ | XBEE_ATCMD_F_WRITE,
#ifdef CONFIG_MODULE_XBEE_ATCMD_HELP
		atcmd78_help,
#endif
	},
	{
		/* "WH" */
		atcmd79_name,
		atcmd79_desc,
		XBEE_ATCMD_F_PARAM_U16 | XBEE_ATCMD_F_READ | XBEE_ATCMD_F_WRITE,
#ifdef CONFIG_MODULE_XBEE_ATCMD_HELP
		atcmd79_help,
#endif
	},
	{
		NULL,
		NULL,
		0,
#ifdef CONFIG_MODULE_XBEE_ATCMD_HELP
		NULL,
#endif
	},
};

const struct xbee_atcmd *xbee_atcmd_lookup_name(const char *atcmd_str)
{
	const struct xbee_atcmd *cmd;
	struct xbee_atcmd copy;

	for (cmd = &xbee_atcmd_list[0], memcpy_P(&copy, cmd, sizeof(copy));
	     copy.name != NULL;
	     cmd++, memcpy_P(&copy, cmd, sizeof(copy))) {

		if (!strcmp_P(atcmd_str, copy.name))
			break;
	}

	if (copy.name == NULL) /* not found */
		return NULL;

	return cmd;
}

const struct xbee_atcmd *xbee_atcmd_lookup_desc(const char *desc)
{
	const struct xbee_atcmd *cmd;
	struct xbee_atcmd copy;

	for (cmd = &xbee_atcmd_list[0], memcpy_P(&copy, cmd, sizeof(copy));
	     copy.name != NULL;
	     cmd++, memcpy_P(&copy, cmd, sizeof(copy))) {
		if (!strcmp_P(desc, copy.desc))
			break;
	}
	if (copy.name == NULL) /* not found */
		return NULL;

	return cmd;
}
