# Makefile for Arduino TTY Mouse Driver
# Compatible with out-of-tree kernel module compilation

# Module name
obj-m := arduino_tty_mouse.o

# Kernel source directory - detect if custom WSL2 kernel exists
WSL2_KERNEL_DIR := /home/$(USER)/WSL2-Linux-Kernel
ifeq ($(wildcard $(WSL2_KERNEL_DIR)/Makefile),)
    KERNEL_DIR ?= /lib/modules/$(shell uname -r)/build
else
    KERNEL_DIR ?= $(WSL2_KERNEL_DIR)
endif

# Current directory
PWD := $(shell pwd)

# Compiler flags for the module
ccflags-y := -Wall -Wextra -std=gnu99

# Default target
all:
	$(MAKE) -C $(KERNEL_DIR) M=$(PWD) modules

# Clean target
clean:
	$(MAKE) -C $(KERNEL_DIR) M=$(PWD) clean
	rm -f Module.markers modules.order

# Install target
install: all
	$(MAKE) -C $(KERNEL_DIR) M=$(PWD) modules_install
	depmod -a

# Load module
load: all
	sudo insmod arduino_tty_mouse.ko

# Unload module
unload:
	-sudo rmmod arduino_tty_mouse

# Reload module
reload: unload load

# Show module info
info:
	modinfo arduino_tty_mouse.ko

# Show driver status
status:
	@echo "=== Module Status ==="
	@lsmod | grep arduino || echo "Module not loaded"
	@echo ""
	@echo "=== Driver Statistics ==="
	@if [ -f /sys/devices/virtual/arduino-tty-mouse/arduino-tty-mouse/stats ]; then \
		cat /sys/devices/virtual/arduino-tty-mouse/arduino-tty-mouse/stats; \
	else \
		echo "Driver not active or sysfs not available"; \
	fi
	@echo ""
	@echo "=== Input Devices ==="
	@cat /proc/bus/input/devices | grep -A 10 "Arduino" || echo "No Arduino input device found"

# Test the driver
test: load
	@echo "=== Testing Arduino TTY Mouse Driver ==="
	@echo "Waiting for driver to stabilize..."
	@sleep 2
	@echo "Enabling mouse..."
	@echo "ENABLE" | sudo tee /sys/devices/virtual/arduino-tty-mouse/arduino-tty-mouse/config > /dev/null
	@sleep 1
	@echo "Sending test command..."
	@echo "TEST" | sudo tee /sys/devices/virtual/arduino-tty-mouse/arduino-tty-mouse/config > /dev/null
	@sleep 2
	@echo "Driver statistics:"
	@cat /sys/devices/virtual/arduino-tty-mouse/arduino-tty-mouse/stats

# Debug - show kernel messages
debug:
	dmesg | grep -i arduino | tail -20

# Help target
help:
	@echo "Arduino TTY Mouse Driver Makefile"
	@echo ""
	@echo "Available targets:"
	@echo "  all      - Build the kernel module (default)"
	@echo "  clean    - Clean build files"
	@echo "  install  - Install module to system"
	@echo "  load     - Load the module"
	@echo "  unload   - Unload the module"
	@echo "  reload   - Unload and load the module"
	@echo "  info     - Show module information"
	@echo "  status   - Show driver status and statistics"
	@echo "  test     - Load module and run basic test"
	@echo "  debug    - Show kernel debug messages"
	@echo "  help     - Show this help message"
	@echo ""
	@echo "Examples:"
	@echo "  make                    # Build the module"
	@echo "  make load               # Load the module"
	@echo "  make test               # Build, load and test"
	@echo "  make status             # Check driver status"
	@echo "  make clean              # Clean build files"

# Custom kernel directory (for cross-compilation or custom kernels)
kernel:
	@echo "Current kernel directory: $(KERNEL_DIR)"
	@echo "Kernel version: $(shell uname -r)"
	@if [ ! -d "$(KERNEL_DIR)" ]; then \
		echo "ERROR: Kernel directory not found!"; \
		echo "Make sure kernel headers are installed:"; \
		echo "  sudo apt install linux-headers-$$(uname -r)"; \
		exit 1; \
	fi

# WSL2 specific target (for custom kernel builds)
wsl2:
	$(MAKE) KERNEL_DIR=/home/$(USER)/WSL2-Linux-Kernel all

# Development targets
dev-setup:
	@echo "Setting up development environment..."
	@echo "Checking dependencies..."
	@which make > /dev/null || (echo "ERROR: make not found" && exit 1)
	@which gcc > /dev/null || (echo "ERROR: gcc not found" && exit 1)
	@ls $(KERNEL_DIR) > /dev/null 2>&1 || (echo "ERROR: Kernel headers not found at $(KERNEL_DIR)" && exit 1)
	@echo "Development environment OK"

# Package target (create distribution archive)
package: clean
	@echo "Creating package..."
	@mkdir -p arduino_tty_mouse_driver
	@cp arduino_tty_mouse.c arduino_tty_mouse_driver/
	@cp Makefile arduino_tty_mouse_driver/
	@echo "README.md\nInstallation:\n  make\n  make load\n\nTesting:\n  make test\n" > arduino_tty_mouse_driver/README.md
	@tar -czf arduino_tty_mouse_driver.tar.gz arduino_tty_mouse_driver/
	@rm -rf arduino_tty_mouse_driver/
	@echo "Package created: arduino_tty_mouse_driver.tar.gz"

.PHONY: all clean install load unload reload info status test debug help kernel wsl2 dev-setup package