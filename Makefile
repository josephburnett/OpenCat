.PHONY : compile
compile:
	arduino-cli compile --fqbn arduino:avr:uno ./OpenCat.ino
