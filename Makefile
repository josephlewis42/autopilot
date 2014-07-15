CC=clang++
#CC=g++
PROJECT_ROOT:=.
BUILD_DIR:=${PROJECT_ROOT}/build
DIST_DIR:=${PROJECT_ROOT}/dist
SRC_PATH:=${PROJECT_ROOT}/src
TESTS_PATH:=$(SRC_PATH)/tests/unittests
SETTINGS_PATH:=$(PROJECT_ROOT)/settings
HEADER_DIRS:=$(shell find $(SRC_PATH) -type d -printf %p\ )
VPATH:=$(shell find $(SRC_PATH) -type d -printf %p:)
GTEST_DIR:=$(PROJECT_ROOT)/extern/gtest


INCLUDE := 	$(addprefix -I,$(HEADER_DIRS)) \
		-I$(PROJECT_ROOT)/../UDenverMavlink/include/ualberta \
		-I$(PROJECT_ROOT)/../UDenverMavlink/include/ \
		-I$(PROJECT_ROOT)/../UDenverMavlink/missionlib \
        -I$(PROJECT_ROOT)/extern/asio/include \
		-I/usr/include/boost \
		-I$(PROJECT_ROOT)/drivers \
		-I$(PROJECT_ROOT)/extern \
        -I$(PROJECT_ROOT)/extern/GeographicLib/include/ \
		-I$(PROJECT_ROOT)/extern/gtest/make \
		-I$(PROJECT_ROOT)/extern/gtest/include \
		-I$(BUILD_DIR)

CFLAGS:=  -pipe -std=c++11 -static ${INCLUDE} -c -g -Wall -Werror 
LDFLAGS:=  -std=c++11  -g -L$(BUILD_DIR) -L/usr/lib -L/usr/include/boost -Lextern/GeographicLib/src -lgtest -lGeographic -lpthread
# DON'T LINK STATIC WHEN USING PTHREADS
# -lboost_thread
SOURCES:=$(shell find $(SRC_PATH) -path $(SRC_PATH)/tests -prune -o -name '*.cc' -printf %f\  )
OBJECTS:=$(patsubst %.cc, $(BUILD_DIR)/%.o, $(SOURCES))
EXECUTABLE=autopilot

all: builddir mavlink $(SOURCES) $(EXECUTABLE) ser2net documentation
	
$(EXECUTABLE): $(OBJECTS) gtest geographiclib
	$(CC) $(OBJECTS) -o ${BUILD_DIR}/$@ $(LDFLAGS) 
	cd $(BUILD_DIR) && ./autopilot test
	
$(BUILD_DIR)/%.o:%.cc
	mkdir -p $(dir $@)
	$(CC) $(CFLAGS) $< -o $@

builddir:
	mkdir -p $(BUILD_DIR)
	cp config.xml $(BUILD_DIR)/config.xml

ser2net: utils/ser2net.cpp
	mkdir -p $(dir $@)
	$(CC) -std=c++11 -g -Wall $< -o ${BUILD_DIR}/$@

mavlink:
	+make --directory ../UDenverMavlink

clean:
	rm -r $(PROJECT_ROOT)/doc
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
	cp -r doc $(DIST_DIR)/doc


documentation:
	doxygen

# Google testing framework
gtest:
	mkdir -p $(BUILD_DIR)
	$(CC) -isystem ${GTEST_DIR}/include -I${GTEST_DIR} \
     		-lpthread -c ${GTEST_DIR}/src/gtest-all.cc -o ${BUILD_DIR}/gtest-all.o
	ar -rv ${BUILD_DIR}/libgtest.a ${BUILD_DIR}/gtest-all.o

geographiclib:
	mkdir -p $(BUILD_DIR)
	+make --directory extern/GeographicLib
