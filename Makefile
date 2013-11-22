CC=g++

PROJECT_ROOT:=/home/joseph/Desktop/Heli/autopilot/trunk/autopilot
BUILD_DIR:=${PROJECT_ROOT}/build
SRC_PATH:=${PROJECT_ROOT}/src
HEADER_DIRS:=$(shell find $(SRC_PATH) -type d -printf %p\ )
VPATH:=$(shell find $(SRC_PATH) -type d -printf %p:)

INCLUDE := 	$(addprefix -I,$(HEADER_DIRS)) \
		-I$(PROJECT_ROOT)/../mavlink/include/ualberta \
		-I$(PROJECT_ROOT)/posixextern

CFLAGS= -m32 ${INCLUDE} -c -Wall
LDFLAGS= -m32 -static -L$(PROJECT_ROOT)/liblinux32 -lboost_thread -lboost_system -lboost_date_time -lboost_filesystem  -lpthread
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
	rm -r ${BUILD_DIR}/
