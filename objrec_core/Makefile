
.PHONY : all cuda cpu debug clean

OBJREC_PKG_PATH=$(shell rospack find objrec_core)
OBJREC_LIB_PATH=$(OBJREC_PKG_PATH)/lib

all: cpu

ObjRecRANSAC_CUDA.tgz: 
	cd $(OBJREC_PKG_PATH) && wget https://github.com/downloads/jhu-lcsr/mvp_objrec/ObjRecRANSAC_CUDA_v1.1.tgz -O ObjRecRANSAC_CUDA.tgz

ObjRecRANSAC_CUDA: ObjRecRANSAC_CUDA.tgz
	cd $(OBJREC_PKG_PATH) && tar -xf *.tgz

ObjRecRANSAC.tgz: 
	cd $(OBJREC_PKG_PATH) && wget https://github.com/downloads/jhu-lcsr/mvp_objrec/ObjRecRANSAC_v1.1.tgz -O ObjRecRANSAC.zip

ObjRecRANSAC: ObjRecRANSAC.tgz
	cd $(OBJREC_PKG_PATH) && tar -xf *.tgz


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

