
OBJREC_PKG_PATH=$(shell rospack find objrec_ransac)
OBJREC_BUILD_DIR=$(OBJREC_PKG_PATH)/ObjRecRANSAC/build

default:=all

all: ObjRecRANSAC
	cd $(OBJREC_BUILD_DIR)  && cmake .. -DCMAKE_BUILD_TYPE=Release && make $(ROS_PARALLEL_JOBS)

debug: ObjRecRANSAC
	cd $(OBJREC_BUILD_DIR) && cmake .. -DCMAKE_BUILD_TYPE=Debug && make $(ROS_PARALLEL_JOBS)

ObjRecRANSAC:
	cd $(OBJREC_PKG_PATH) && unzip *.zip

clean:
	rm -rf ObjRecRANSAC
