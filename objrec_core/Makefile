
.PHONY : all cuda cpu debug clean

OBJREC_PKG_PATH=$(shell rospack find objrec_core)
OBJREC_LIB_PATH=$(OBJREC_PKG_PATH)/lib

TARBALL_BASE_URI=https://github.com/downloads/tum-mvp/mvp_objrec
TARBALL_VERSION=v1.1



all: cpu

%.tgz: 
	cd $(OBJREC_PKG_PATH) && wget $(TARBALL_BASE_URI)/$@

ObjRecRANSA%: ObjRecRANSA%_$(TARBALL_VERSION).tgz
	cd $(OBJREC_PKG_PATH) && tar -xf $<.tgz



cuda: ObjRecRANSAC_CUDA
	$(eval OBJREC_BUILD_DIR=$(OBJREC_PKG_PATH)/ObjRecRANSAC_CUDA/build)
	mkdir -p $(OBJREC_BUILD_DIR)
	cd $(OBJREC_BUILD_DIR)  && cmake .. -DUSE_CUDA:BOOL=True -DCMAKE_BUILD_TYPE=Debug && make $(ROS_PARALLEL_JOBS)
	mkdir -p $(OBJREC_LIB_PATH)
	cp $(OBJREC_BUILD_DIR)/src/BasicTools/libBasicTools.* $(OBJREC_LIB_PATH)
	cp $(OBJREC_BUILD_DIR)/src/BasicToolsL1/libBasicToolsL1.* $(OBJREC_LIB_PATH)
	cp $(OBJREC_BUILD_DIR)/src/ObjRecRANSAC/libObjRecRANSAC.* $(OBJREC_LIB_PATH)
	cp $(OBJREC_BUILD_DIR)/src/VtkBasics/libVtkBasics.* $(OBJREC_LIB_PATH)	

cpu: ObjRecRANSAC
	$(eval OBJREC_BUILD_DIR=$(OBJREC_PKG_PATH)/ObjRecRANSAC/build)
	mkdir -p $(OBJREC_BUILD_DIR)
	cd $(OBJREC_BUILD_DIR)  && cmake .. -DCMAKE_BUILD_TYPE=Debug && make $(ROS_PARALLEL_JOBS)
	mkdir -p $(OBJREC_LIB_PATH)
	cp $(OBJREC_BUILD_DIR)/src/BasicTools/libBasicTools.* $(OBJREC_LIB_PATH)
	cp $(OBJREC_BUILD_DIR)/src/BasicToolsL1/libBasicToolsL1.* $(OBJREC_LIB_PATH)
	cp $(OBJREC_BUILD_DIR)/src/ObjRecRANSAC/libObjRecRANSAC.* $(OBJREC_LIB_PATH)
	cp $(OBJREC_BUILD_DIR)/src/VtkBasics/libVtkBasics.* $(OBJREC_LIB_PATH)	

debug: ObjRecRANSAC
	$(eval OBJREC_BUILD_DIR=$(OBJREC_PKG_PATH)/ObjRecRANSAC/build)
	mkdir -p $(OBJREC_BUILD_DIR)
	cd $(OBJREC_BUILD_DIR) && cmake .. -DCMAKE_BUILD_TYPE=Debug && make $(ROS_PARALLEL_JOBS)
	mkdir -p $(OBJREC_LIB_PATH)
	cp $(OBJREC_BUILD_DIR)/src/BasicTools/libBasicTools.* $(OBJREC_LIB_PATH)
	cp $(OBJREC_BUILD_DIR)/src/BasicToolsL1/libBasicToolsL1.* $(OBJREC_LIB_PATH)
	cp $(OBJREC_BUILD_DIR)/src/ObjRecRANSAC/libObjRecRANSAC.* $(OBJREC_LIB_PATH)
	cp $(OBJREC_BUILD_DIR)/src/VtkBasics/libVtkBasics.* $(OBJREC_LIB_PATH)	

clean:
	rm -rf ObjRecRANSAC/build
	rm -rf libs

