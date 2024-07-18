# Define variables for common commands and options
PIO_REMOTE = pio remote
DEVICE_LIST = $(PIO_REMOTE) device list --json-output
JQ_FILTER = jq -c '.oknotok.[] | select(.description | contains("Serial")) | .port'
XARGS_CMD = xargs -I % $(PIO_REMOTE) run -t upload --upload-port %
DEFAULT_BAUD = 115200

# Phony targets to ensure make does not confuse these with actual files
.PHONY: rlist rup rlisten

# List all remote devices
rlist:
	$(DEVICE_LIST)

# Upload to all remote devices with 'Serial' in their description
rup:
	$(DEVICE_LIST) | $(JQ_FILTER) | $(XARGS_CMD)

# Upload to all remote devices with 'Serial' in their description in listen-only mode
rlisten:
	$(DEVICE_LIST) | $(JQ_FILTER) | $(XARGS_CMD) -e listen-only

rmonitor:
	$(PIO_REMOTE) device monitor -b $(DEFAULT_BAUD)


