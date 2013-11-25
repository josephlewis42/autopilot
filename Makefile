CC=g++

PROJECT_ROOT:=.
BUILD_DIR:=${PROJECT_ROOT}/build
DIST_DIR:=${PROJECT_ROOT}/dist
SRC_PATH:=${PROJECT_ROOT}/src
SETTINGS_PATH:=$(PROJECT_ROOT)/settings
HEADER_DIRS:=$(shell find $(SRC_PATH) -type d -printf %p\ )
VPATH:=$(shell find $(SRC_PATH) -type d -printf %p:)

INCLUDE := 	$(addprefix -I,$(HEADER_DIRS)) \
		-I$(PROJECT_ROOT)/../mavlink/include/ualberta \
		-I$(PROJECT_ROOT)/extern/Linux \
		-I$(PROJECT_ROOT)/extern/mavlink/include/ualberta

CFLAGS= -m32 ${INCLUDE} -c -Wall -g
LDFLAGS= -g -m32 -static -L/usr/lib -L$(PROJECT_ROOT)/lib/Linux32 -lboost_thread -lboost_system -lboost_date_time -lboost_filesystem  -lpthread
SOURCES:=$(shell find $(SRC_PATH) -path $(SRC_PATH)/tests -prune -o -name '*.cc' -printf %f\  )
OBJECTS:=$(patsubst %.cc, $(BUILD_DIR)/%.o, $(SOURCES))
EXECUTABLE=autopilot


all: $(SOURCES) $(EXECUTABLE)
	
$(EXECUTABLE): $(OBJECTS) 
	$(CC) $(OBJECTS) -o ${BUILD_DIR}/$@ $(LDFLAGS) 

$(BUILD_DIR)/%.o:%.cc
	mkdir -p $(dir $@)
	$(CC) $(CFLAGS) $< -o $@

clean:
	rm -r $(BUILD_DIR)
	rm -r $(DIST_DIR)

dist: all
	mkdir -p $(DIST_DIR)
	cp $(BUILD_DIR)/$(EXECUTABLE) $(DIST_DIR)
	cp $(DIST_DIR)/$(EXECUTABLE) $(DIST_DIR)/$(EXECUTABLE)_g
	strip $(DIST_DIR)/$(EXECUTABLE)
	cp $(SETTINGS_PATH)/* $(DIST_DIR)
	
