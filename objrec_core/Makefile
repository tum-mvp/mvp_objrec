
.PHONY : all cuda cpu debug clean

OBJREC_PKG_PATH=$(shell rospack find objrec_core)
OBJREC_BUILD_DIR=$(OBJREC_PKG_PATH)/ObjRecRANSAC/build
OBJREC_LIB_PATH=$(OBJREC_PKG_PATH)/lib

all: cpu

ObjRecRANSAC.zip: 
	wget https://github.com/downloads/jhu-lcsr/mvp_objrec/ObjRecRANSAC_v1.1.zip -O ObjRecRANSAC.zip

ObjRecRANSAC: ObjRecRANSAC.zip
	#cd $(OBJREC_PKG_PATH) && tar -xf *.tar
	cd $(OBJREC_PKG_PATH) && unzip *.zip

cuda: ObjRecRANSAC
	mkdir -p $(OBJREC_BUILD_DIR)
	cd $(OBJREC_BUILD_DIR)  && cmake .. -DUSE_CUDA:BOOL=True -DCMAKE_BUILD_TYPE=Debug && make $(ROS_PARALLEL_JOBS)
	mkdir -p $(OBJREC_LIB_PATH)
	cp $(OBJREC_BUILD_DIR)/src/BasicTools/libBasicTools.* $(OBJREC_LIB_PATH)
	cp $(OBJREC_BUILD_DIR)/src/BasicToolsL1/libBasicToolsL1.* $(OBJREC_LIB_PATH)
	cp $(OBJREC_BUILD_DIR)/src/ObjRecRANSAC/libObjRecRANSAC.* $(OBJREC_LIB_PATH)
	cp $(OBJREC_BUILD_DIR)/src/VtkBasics/libVtkBasics.* $(OBJREC_LIB_PATH)	

cpu: ObjRecRANSAC
	mkdir -p $(OBJREC_BUILD_DIR)
	cd $(OBJREC_BUILD_DIR)  && cmake .. -DCMAKE_BUILD_TYPE=Debug && make $(ROS_PARALLEL_JOBS)
	mkdir -p $(OBJREC_LIB_PATH)
	cp $(OBJREC_BUILD_DIR)/src/BasicTools/libBasicTools.* $(OBJREC_LIB_PATH)
	cp $(OBJREC_BUILD_DIR)/src/BasicToolsL1/libBasicToolsL1.* $(OBJREC_LIB_PATH)
	cp $(OBJREC_BUILD_DIR)/src/ObjRecRANSAC/libObjRecRANSAC.* $(OBJREC_LIB_PATH)
	cp $(OBJREC_BUILD_DIR)/src/VtkBasics/libVtkBasics.* $(OBJREC_LIB_PATH)	

debug: ObjRecRANSAC
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

