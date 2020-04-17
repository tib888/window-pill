rem * This 'set HOME' below used to tell GDB where is the home directory.
rem * In that directory there must be a .gdbinit file with this content:
rem * 	set auto-load safe-path <targetpath>\.gdbinit
rem * This allows gdb to read the .gdbinit command file from the <targetpath>
rem * That file may contain something like this:
rem * 	target remote :3333
rem * 	monitor arm semihosting enable
rem * 	load
rem * 	continue
rem * This makes the connection to the OCD if it was started earlier with a command like this:
rem * 	openocd -f "stlink-v2.cfg" -f "stm32f1x.cfg"
rem * Then loads the binary on the HW and executes it.

rem set HOME=%HOMEPATH%
rem set HOME=%USERPROFILE%
rem itmdump -f itm.fifo -s 0

cargo run --release --target thumbv7m-none-eabi
