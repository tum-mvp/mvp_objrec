
OBJREC_PKG_PATH=$(shell rospack find objrec_ransac)
OBJREC_BUILD_DIR=$(OBJREC_PKG_PATH)/ObjRecRANSAC/build
OBJREC_LIB_PATH=$(OBJREC_PKG_PATH)/lib

default:=all

all: ObjRecRANSAC
	mkdir -p $(OBJREC_BUILD_DIR)
	cd $(OBJREC_BUILD_DIR)  && cmake .. -DCMAKE_BUILD_TYPE=Debug && make $(ROS_PARALLEL_JOBS)
	mkdir -p $(OBJREC_LIB_PATH)
	cp $(OBJREC_BUILD_DIR)/src/BasicTools/libBasicTools.so $(OBJREC_LIB_PATH)/libBasicTools.so	
	cp $(OBJREC_BUILD_DIR)/src/BasicToolsL1/libBasicToolsL1.so $(OBJREC_LIB_PATH)/libBasicToolsL1.so	
	cp $(OBJREC_BUILD_DIR)/src/ObjRecRANSAC/libObjRecRANSAC.so $(OBJREC_LIB_PATH)/libObjRecRANSAC.so	
	cp $(OBJREC_BUILD_DIR)/src/VtkBasics/libVtkBasics.so $(OBJREC_LIB_PATH)/libVtkBasics.so	

debug: ObjRecRANSAC
	mkdir -p $(OBJREC_BUILD_DIR)
	cd $(OBJREC_BUILD_DIR) && cmake .. -DCMAKE_BUILD_TYPE=Debug && make $(ROS_PARALLEL_JOBS)
	mkdir -p $(OBJREC_LIB_PATH)
	cp $(OBJREC_BUILD_DIR)/src/BasicTools/libBasicTools.so $(OBJREC_LIB_PATH)/libBasicTools.so	
	cp $(OBJREC_BUILD_DIR)/src/BasicToolsL1/libBasicToolsL1.so $(OBJREC_LIB_PATH)/libBasicToolsL1.so	
	cp $(OBJREC_BUILD_DIR)/src/ObjRecRANSAC/libObjRecRANSAC.so $(OBJREC_LIB_PATH)/libObjRecRANSAC.so	
	cp $(OBJREC_BUILD_DIR)/src/VtkBasics/libVtkBasics.so $(OBJREC_LIB_PATH)/libVtkBasics.so	

ObjRecRANSAC:
	cd $(OBJREC_PKG_PATH) && tar -xf *.tar

clean:
	rm -rf ObjRecRANSAC/build
	rm -rf libs
