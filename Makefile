CC=g++

PROJECT_ROOT:=.
BUILD_DIR:=${PROJECT_ROOT}/build
DIST_DIR:=${PROJECT_ROOT}/dist
SRC_PATH:=${PROJECT_ROOT}/src
TESTS_PATH:=$(SRC_PATH)/tests/unittests
SETTINGS_PATH:=$(PROJECT_ROOT)/settings
HEADER_DIRS:=$(shell find $(SRC_PATH) -type d -printf %p\ )
VPATH:=$(shell find $(SRC_PATH) -type d -printf %p:)

INCLUDE := 	$(addprefix -I,$(HEADER_DIRS)) \
		-I$(PROJECT_ROOT)/../mavlink/include/ualberta \
		-I$(PROJECT_ROOT)/extern/mavlink/include/ualberta \
		-I/usr/include/boost
		#-I$(PROJECT_ROOT)/extern/Linux \
#32 bit
#CFLAGS:=  -m32 -static ${INCLUDE} -c -g -Wall -fstack-protector-all
#LDFLAGS:= -m32 -static -g -L/usr/lib -L$(PROJECT_ROOT)/lib/Linux32 -fstack-protector-all -lboost_thread -lboost_system -lboost_date_time -lboost_filesystem  -lpthread
CFLAGS:=  -std=c++11 -static ${INCLUDE} -c -g -Wall
LDFLAGS:=  -std=c++11 -static -g -L/usr/lib -L/usr/include/boost -lboost_thread -lboost_system -lboost_date_time -lboost_filesystem  -lpthread
# -L$(PROJECT_ROOT)/lib/Linux64

SOURCES:=$(shell find $(SRC_PATH) -path $(SRC_PATH)/tests -prune -o -name '*.cc' -printf %f\  )
OBJECTS:=$(patsubst %.cc, $(BUILD_DIR)/%.o, $(SOURCES))
EXECUTABLE=autopilot

all: $(SOURCES) $(EXECUTABLE) ser2net
	
$(EXECUTABLE): $(OBJECTS) 
	$(CC) $(OBJECTS) -o ${BUILD_DIR}/$@ $(LDFLAGS) 

$(BUILD_DIR)/%.o:%.cc
	mkdir -p $(dir $@)
	$(CC) $(CFLAGS) $< -o $@

ser2net: utils/ser2net.cpp
	mkdir -p $(dir $@)
	$(CC) -std=c++11 -g -Wall $< -o ${BUILD_DIR}/$@

clean:
	rm -r $(BUILD_DIR)
	rm -r $(DIST_DIR)

install:
	cp $(BUILD_DIR)/ser2net /usr/local/bin
	cp $(BUILD_DIR)/$(EXECUTABLE) /usr/local/bin
	

dist: all
	mkdir -p $(DIST_DIR)
	cp $(BUILD_DIR)/$(EXECUTABLE) $(DIST_DIR)
	cp $(DIST_DIR)/$(EXECUTABLE) $(DIST_DIR)/$(EXECUTABLE)_g
	strip $(DIST_DIR)/$(EXECUTABLE)
	cp $(SETTINGS_PATH)/* $(DIST_DIR)

tests: $(EXECUTABLE)
	$(CC) $(CFLAGS) $(TESTS_PATH)/* -o ${BUILD_DIR}/test.o
	$(CC) $(OBJECTS) -o ${BUILD_DIR}/test.o $(LDFLAGS) 

