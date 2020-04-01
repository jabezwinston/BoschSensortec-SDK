###########################################################################################
################################### General Information ###################################
###########################################################################################

The command-line interface tool bhy2cli can be used to communicate to BHI260/BHA260 by using predefined commands, which are translated to corresponding host interface commands.
A usage information can be found below.


###########################################################################################
###################################        Usage        ###################################
###########################################################################################

A bhy2cli call consists of one or more command options with their respective parameters (if they have additional parameters). The different command options can be used successively.

Call Syntax: bhy2cli <Option 1 [<parameters 1>]> <Option 2 [<parameters 2>]> ... <Option N [<parameters N>]>  

Available command options:

-h                                                    = Print this usage message.
-i                                                    = Show device information: Device ID, ROM version, RAM version, Power state, list of available sensors,
                                                        content of Boot Status register, content of Error value register.
-b <firmware path>                                    = Reset, upload specified firmware to RAM and boot from RAM [equivalent to using -n -u -g successively].
-n                                                    = Reset sensor hub.
-a <sensor_type>:<sensor_name>:<fifo_type>:<total_output_payload_in_bytes>:<output_format_0>:<output_format_1>...
                                                      = Register the expected payload of a new custom virtual sensor,
                                                        including information about the FIFO (fifo_type), which contains data produced by this sensor.
                                                          -Valid fifo_types: wk: Wake up, nw: Non-Wake up
                                                          -Valid output_formats: u8: Unsigned 8 Bit, u16: Unsigned 16 Bit, u32: Unsigned 32 Bit, s8: Signed 8 Bit,
                                                                              s16: Signed 16 Bit, s32: Signed 32 Bit, f: Float, c: Char 
                                                          -e.g.: -a 160:"Lean Orientation":nw:2:c:c 
                                                          -Note that the corresponding virtual sensor has to be enabled in the same function call (trailing -c option),
                                                           since the registration of the sensor is temporary. 
-r <adr>[:<len>]                                      = Read from register address <adr> for length <len> bytes.
                                                          -If input <len> is not provided, the default read length is 1 byte.
                                                          -When reading registers with auto-increment, the provided register as well as the following registers will be read.
                                                          -e.g -r 0x08:3 will read the data of registers 0x08, 0x09 and 0x0a.
                                                        max. 53 bytes can be read at once.
-w <adr>=<val1>[,<val2>]...                           = Write to register address <adr> with comma separated values <val>.
                                                          -If more values provided <val>, the additional
                                                           values will be written to the following addresses.
                                                          -When writing to registers with auto-increment, the provided register as well as the following registers will be written.
                                                       -e.g -w 0x08=0x02,0x03,0x04 will write the provided data to registers 0x08, 0x09 and 0x0a.
                                                        max. 46 bytes can be written at once.
-s <param_id>                                         = Display read_param response of parameter <param_id>.
-t <param_id>=<val1>[,<val2>]...                      = Write data to parameter <param_id> with the bytes to be written, <val1>[,<val2>]... .
                                                        -e.g. 0x103=5,6 will write 0x05 to the first byte and 0x06 to the second byte of the parameter "Fifo Control"
-u <firmware path>                                    = Upload firmware to RAM.
-f <firmware path>                                    = Upload firmware to external-flash.
-g <medium>                                           = Boot from the specified <medium>: "f" for FLASH, "r" for RAM.
-e                                                    = Erase external-flash.
-c <sensor_type>:<frequency>[:<latency>]              = Activate sensor <sensor_type> at specified sample rate <frequency>,
                                                           = latency <latency>, duration time <time>, sample counts <count>.
                                                               - At least <frequency> is a must input parameter.
                                                               - <latency> is optional.
                                                               - User can active more than one sensor at the same time by repeating -c option.
                                                               - id: sensor id.
                                                               - frequency(Hz): sensor ODR.
                                                               - latency(ms): sensor data outputs with a latency.