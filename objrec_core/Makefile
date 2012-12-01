
.PHONY : all cuda cpu debug clean

OBJREC_PKG_PATH=$(shell rospack find objrec_core)
OBJREC_LIB_PATH=$(OBJREC_PKG_PATH)/lib

TARBALL_BASE_URI=https://github.com/downloads/tum-mvp/mvp_objrec
TARBALL_VERSION=1.3

OBJREC_SOURCE_DIR=ObjRecRANSAC-$(TARBALL_VERSION)
OBJREC_BUILD_DIR=$(OBJREC_PKG_PATH)/$(OBJREC_SOURCE_DIR)/build

all: cpu

%.tgz: 
	cd $(OBJREC_PKG_PATH) && wget $(TARBALL_BASE_URI)/$@

$(OBJREC_SOURCE_DIR): ObjRecRANSAC-$(TARBALL_VERSION).tgz
	cd $(OBJREC_PKG_PATH) && tar -xf $<

cuda: $(OBJREC_SOURCE_DIR)
	mkdir -p $(OBJREC_BUILD_DIR)
	cd $(OBJREC_BUILD_DIR)  && cmake .. -DUSE_CUDA:BOOL=True -DCMAKE_BUILD_TYPE=Debug && make $(ROS_PARALLEL_JOBS)
	mkdir -p $(OBJREC_LIB_PATH)
	cp $(OBJREC_BUILD_DIR)/src/BasicTools/libBasicTools.* $(OBJREC_LIB_PATH)
	cp $(OBJREC_BUILD_DIR)/src/BasicToolsL1/libBasicToolsL1.* $(OBJREC_LIB_PATH)
	cp $(OBJREC_BUILD_DIR)/src/ObjRecRANSAC/libObjRecRANSAC.* $(OBJREC_LIB_PATH)
	cp $(OBJREC_BUILD_DIR)/src/VtkBasics/libVtkBasics.* $(OBJREC_LIB_PATH)	

cpu: $(OBJREC_SOURCE_DIR)
	mkdir -p $(OBJREC_BUILD_DIR)
	cd $(OBJREC_BUILD_DIR)  && cmake .. -DCMAKE_BUILD_TYPE=Debug && make $(ROS_PARALLEL_JOBS)
	mkdir -p $(OBJREC_LIB_PATH)
	cp $(OBJREC_BUILD_DIR)/src/BasicTools/libBasicTools.* $(OBJREC_LIB_PATH)
	cp $(OBJREC_BUILD_DIR)/src/BasicToolsL1/libBasicToolsL1.* $(OBJREC_LIB_PATH)
	cp $(OBJREC_BUILD_DIR)/src/ObjRecRANSAC/libObjRecRANSAC.* $(OBJREC_LIB_PATH)
	cp $(OBJREC_BUILD_DIR)/src/VtkBasics/libVtkBasics.* $(OBJREC_LIB_PATH)	

debug: $(OBJREC_SOURCE_DIR)
	mkdir -p $(OBJREC_BUILD_DIR)
	cd $(OBJREC_BUILD_DIR) && cmake .. -DCMAKE_BUILD_TYPE=Debug && make $(ROS_PARALLEL_JOBS)
	mkdir -p $(OBJREC_LIB_PATH)
	cp $(OBJREC_BUILD_DIR)/src/BasicTools/libBasicTools.* $(OBJREC_LIB_PATH)
	cp $(OBJREC_BUILD_DIR)/src/BasicToolsL1/libBasicToolsL1.* $(OBJREC_LIB_PATH)
	cp $(OBJREC_BUILD_DIR)/src/ObjRecRANSAC/libObjRecRANSAC.* $(OBJREC_LIB_PATH)
	cp $(OBJREC_BUILD_DIR)/src/VtkBasics/libVtkBasics.* $(OBJREC_LIB_PATH)	

clean:
	rm -rf ObjRecRANSAC/build
	rm -rf ObjRecRANSAC_CUDA/build
	rm -rf libs

