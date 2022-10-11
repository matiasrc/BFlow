#include "ofApp.h"

using namespace ofxCv;
using namespace cv;

void ofApp::setup() {
    ofSetFrameRate(60);
    ofSetVerticalSync(true);
    
    //----------------- XML -------------------
    loadSettings();
    
    //----------------- CAM -------------------
    //viewCam            = false;  // en el XML
    camWidth            = 640;
    camHeight           = 480;
    imagePixelNumber    = camWidth * camHeight;
    //deviceID            = 1;
    deviceName          = "NO DEVICE AVAILABLE";

    //hMirror             = false; // en el XML
    //vMirror             = false; // en el XML
       
    isOneDeviceAvailable =  false;
    
    setupCam(deviceID);
    
    //----------------- WARP -------------------
     warpON =  false;
     cualPunto = 0;
    
     moverPunto = false;
     
     mirroredImg.allocate(camWidth,camHeight);
     warpedImg.allocate(camWidth,camHeight);
    
    //----------------- FLOW -------------------
    MIN_SIZE = 20;
    drawRect = false;
    //----------------- GUI -------------------
    //required call
    gui.setup();
    
    ImGui::GetIO().MouseDrawCursor = false;
        
    //----------------- OSC -------------------
    
    sender.setup(host, puertoOUT);
    receiver.setup(puertoIN);
}

void ofApp::update() {
    cam.update();
    
    float w = ofGetWidth();
    float h = ofGetHeight();
    
    float fw = flow.getWidth();
    float fh = ofGetHeight();
    
    if(cam.isFrameNew()) {
        
        mirroredImg.setFromPixels(cam.getPixels());
        mirroredImg.mirror(vMirror, hMirror);
        
        warpedImg = mirroredImg;
        warpedImg.warpPerspective(warp[0], warp[1], warp[2], warp[3]);
        
        flow.setPyramidScale(fbPyrScale);
        flow.setNumLevels(fbLevels);
        flow.setWindowSize(fbWinSize);
        flow.setNumIterations(fbIterations);
        flow.setPolyN(fbPolyN);
        flow.setPolySigma(fbPolySigma);
        flow.setUseGaussian(fbUseGaussian);
        
        flow.calcOpticalFlow(warpedImg);

        // -------- OSC---------
        glm::vec2 totalFlow = flow.getTotalFlow();
        vector <float> totalFlowData;
        totalFlowData.push_back(totalFlow.x);
        totalFlowData.push_back(totalFlow.y);
        enviarOsc(etiquetaTotalFlow, totalFlowData);
        
        glm::vec2 averageFlow = flow.getAverageFlow();
        vector <float> averagelFlowData;
        averagelFlowData.push_back(averageFlow.x);
        averagelFlowData.push_back(averageFlow.y);
        enviarOsc(etiquetaAverageFlow, averagelFlowData);
        
        int totalAreas = areas.size();
        
        for(int i=0; i<totalAreas; i++){
            
            vector <float> totalFlowInRegionData;
            vector <float> averageFlowInRegionData;
            
            ofRectangle r = areas[i];
            glm::vec2 totalFlowInRegion = flow.getTotalFlowInRegion(r);
            glm::vec2 averageFlowInRegion = flow.getAverageFlowInRegion(r);
            
            float rx = r.x / camWidth;
            float ry = r.y / camHeight;
            float rw = r.width / camWidth;
            float rh = r.height / camHeight;
            
            totalFlowInRegionData.push_back(float(i));              // 0
            totalFlowInRegionData.push_back(totalFlowInRegion.x);   // 1
            totalFlowInRegionData.push_back(totalFlowInRegion.y);   // 2
            totalFlowInRegionData.push_back(rx);                    // 3
            totalFlowInRegionData.push_back(ry);                    // 4
            totalFlowInRegionData.push_back(rw);                    // 5
            totalFlowInRegionData.push_back(rh);                    // 6
            
            averageFlowInRegionData.push_back(float(i));
            averageFlowInRegionData.push_back(averageFlowInRegion.x);
            averageFlowInRegionData.push_back(averageFlowInRegion.y);
            averageFlowInRegionData.push_back(rx);
            averageFlowInRegionData.push_back(ry);
            averageFlowInRegionData.push_back(rw);
            averageFlowInRegionData.push_back(rh);
            
            enviarOsc(etiquetaTotalFlowInRegion, totalFlowInRegionData);
            enviarOsc(etiquetaAverageFlowInRegion, averageFlowInRegionData);
        }
    }
    
    recibirOsc();
    
    while(inMessages.size() > 0){
        
        ofxOscMessage m = inMessages[0];
        
        if(m.getAddress() == etiquetaToRegion){

            if(m.getNumArgs() == 5){
        
                // both the arguments are floats
                float id = m.getArgAsFloat(0);
                float rx = m.getArgAsFloat(1);
                float ry = m.getArgAsFloat(2);
                float rw = m.getArgAsFloat(3);
                float rh = m.getArgAsFloat(4);
                
                vector <float> averageFlowInRegionData;
                
                ofRectangle r = ofRectangle(rx * camWidth, ry * camHeight, rw * camWidth, rh * camHeight);
                glm::vec2 averageFlowInRegion = flow.getAverageFlowInRegion(r);
                
                averageFlowInRegionData.push_back(float(id));
                averageFlowInRegionData.push_back(averageFlowInRegion.x);
                averageFlowInRegionData.push_back(averageFlowInRegion.y);
                
                enviarOsc(etiquetaFromRegion, averageFlowInRegionData);
            }
        }
        else if(m.getAddress() == etiquetaToPosition){

            if(m.getNumArgs() == 3){
                
                // arguments are floats
                float id = m.getArgAsFloat(0);
                float x = m.getArgAsFloat(1);
                float y = m.getArgAsFloat(2);
                                
                vector <float> flowInPositionData;
                
                glm::vec2 flowInPosition = flow.getFlowOffset(x*camWidth , y*camHeight);
                
                flowInPositionData.push_back(float(id));
                flowInPositionData.push_back(flowInPosition.x);
                flowInPositionData.push_back(flowInPosition.y);
                
                enviarOsc(etiquetaFromPosition, flowInPositionData);
            }
        }
        inMessages.erase(inMessages.begin());
    }
    
    if(deleteAreas){
        bool pressed = ofGetMousePressed();
        float mx, my = -1.0;
        if(pressed){
            mx = ofMap(ofGetMouseX(), 0, ofGetWidth(), 0, fw);
            my = ofMap(ofGetMouseY(), 0, ofGetHeight(), 0, fh);
            for(int i=areas.size() -1; i>=0; i--){
                if(areas.at(i).inside(mx, my)){
                    areas.erase(areas.begin() + i);
                }
            }
        }
    }
    warpingReset();
}

void ofApp::draw() {
    
    float w = ofGetWidth();
    float h = ofGetHeight();
    float fw = flow.getWidth();
    float fh = flow.getHeight();
        
    if(imageView == 0){
        mirroredImg.draw(0, 0, w, h);
    }
    else if(imageView == 1){
        warpedImg.draw(0, 0, w, h);
    }
    flow.draw(0,0,w,h);
    
    ofPushStyle();
    ofNoFill();
    if(deleteAreas){
        ofSetColor(255, 0, 0);
    }else{
        ofSetColor(0, 255, 0);
    }
    
    if(drawRect){
        float rx = rect.x / camWidth * w;
        float ry = rect.y / camHeight * h;
        float rw = rect.width / camWidth * w;
        float rh = rect.height / camHeight * h;
        ofDrawRectangle(rx, ry, rw, rh);
    }
    
    for(int i=0; i<areas.size(); i++){
        
        float rx = areas.at(i).x / camWidth * w;
        float ry = areas.at(i).y / camHeight * h;
        float rw = areas.at(i).width / camWidth * w;
        float rh = areas.at(i).height / camHeight * h;
        ofDrawRectangle(rx, ry, rw, rh);
        ofDrawBitmapStringHighlight("area: " + ofToString(i), rx, ry);
    }
    ofPopStyle();
    
    if(warpON){
        ofPushStyle();
        ofFill();
        ofPolyline pl;
        
        float cornerSize = 15;
        
        for(int i=0; i<4; i++){
            float x = warp[i].x / camWidth * w;
            float y = (warp[i].y / camHeight * h);
        
            pl.addVertex(x, y);
            
            corner[i].setFromCenter(x, y, cornerSize, cornerSize);
            
            ofFill();
            //ofDrawCircle(x, y, 5);
            if(i == cualPunto){
                ofSetColor(255, 0, 0);
            }else{
                ofSetColor(0, 255, 255);
            }
            ofDrawRectangle(corner[i]);
        }
        ofSetColor(0, 255, 255);
        pl.close();
        pl.draw();
        ofPopStyle();
    }
    
    if(deleteAreas){
        ofPushStyle();
        ofDrawBitmapStringHighlight("Borrado de zonas activado, presionad tecla d para salir", 5, ofGetHeight() - 5, ofColor(255,0, 0));
        ofPopStyle();
    }
    if(warpON){
        ofPushStyle();
        ofDrawBitmapStringHighlight("Deformación de entrada activada, presionad tecla w para salir", 5, ofGetHeight() - 25, ofColor(255,0, 0));
        ofPopStyle();
    }
    
    drawGui();
    
    ofSetWindowTitle("FPS: " + ofToString(ofGetFrameRate()));
}

void ofApp::setupCam(int devID){
    
    wdevices = cam.listDevices();
    for(int i=0;i<static_cast<int>(wdevices.size());i++){
        if(wdevices[i].bAvailable){
            isOneDeviceAvailable = true;
            devicesVector.push_back(wdevices[i].deviceName);
            devicesID.push_back(i);

            for(size_t f=0;f<wdevices[i].formats.size();f++){
                ofLog(OF_LOG_NOTICE,"Capture Device format vailable: %ix%i",wdevices[i].formats.at(f).width,wdevices[i].formats.at(f).height);
            }
        }
    }
    
    cam.setDeviceID(devID);
    cam.setup(camWidth, camHeight);
}
void ofApp::resetCameraSettings(int devID){
    if(devID!=deviceID){
        ofLog(OF_LOG_NOTICE,"Changing Device to: %s",devicesVector[devID].c_str());

        deviceID = devID;
        deviceName = devicesVector[deviceID];
    }
    
    if(cam.isInitialized()){
        cam.close();
    }
    //cam = new ofVideoGrabber();
    cam.setDeviceID(deviceID);
    cam.setup(camWidth, camHeight);
}

//--------------------------------------------------------------
void ofApp::enviarOsc(string etiqueta, float valor){
    ofxOscMessage m;
    m.setAddress(etiqueta);
    m.addFloatArg(valor);
    sender.sendMessage(m, false);
}
//--------------------------------------------------------------
void ofApp::enviarOsc(string etiqueta, vector<float> valores){
    ofxOscMessage m;
    m.setAddress(etiqueta);
    for( int i=0; i<valores.size(); i++){
        m.addFloatArg(valores[i]);
    }
    sender.sendMessage(m, false);
}
//--------------------------------------------------------------
void ofApp::recibirOsc(){
    
    while(receiver.hasWaitingMessages()){
        // get the next message
        ofxOscMessage m;
        receiver.getNextMessage(m);
        inMessages.push_back(m);
    }
}
//--------------------------------------------------------------
void ofApp::loadSettings(){
    //-----------
    //the string is printed at the top of the app
    //to give the user some feedback
    xmlMessage = "loading mySettings.xml";

    //we load our settings file
    //if it doesn't exist we can still make one
    //by hitting the 's' key
    if( XML.loadFile("mySettings.xml") ){
        xmlMessage = "mySettings.xml loaded!";
    }else{
        xmlMessage = "unable to load mySettings.xml check data/ folder";
    }

    //read the colors from XML
    //if the settings file doesn't exist we assigns default values (170, 190, 240)
    //red        = XML.getValue("BACKGROUND:COLOR:RED", 170);
    
    //---------------- OSC --------------------
    puertoOUT = XML.getValue("OSC:PUERTOOUT", 3333);
    puertoIN = XML.getValue("OSC:PUERTOIN", 4444);
    host = XML.getValue("OSC:HOST", "127.0.0.1");
    
    etiquetaTotalFlow = XML.getValue("OSC:ETIQUETA:TOTALFLOW", "/bflow/totalflow");
    enviarTotalFlow = XML.getValue("OSC:ENVIARTOTALFLOW", true);
    
    etiquetaAverageFlow = XML.getValue("OSC:ETIQUETA:AVERAGEFLOW", "/bflow/averageflow");
    enviarAverageFlow = XML.getValue("OSC:ENVIARAVERAGEFLOW", true);
    
    etiquetaTotalFlowInRegion = XML.getValue("OSC:ETIQUETA:TOTALFLOWINREGION", "/bflow/totalflowinregion");
    enviarTotalFlowInRegion = XML.getValue("OSC:ENVIARTOTALFLOWINREGION", true);
    
    etiquetaAverageFlowInRegion = XML.getValue("OSC:ETIQUETA:AVERAGEFLOWINREGION", "/bflow/averageflowinregion");
    enviarAverageFlowInRegion = XML.getValue("OSC:ENVIARAVERAGEFLOWINREGION", true);
    
    etiquetaToRegion = XML.getValue("OSC:ETIQUETA:TOREGION", "/tobflow/flowInRegion");
    enviarFlowInRegion = XML.getValue("OSC:ENVIARFLOWINREGION", true);
    
    etiquetaToPosition = XML.getValue("OSC:ETIQUETA:TOPOSITION", "/tobflow/flowInPosition");
    enviarFlowInPosition = XML.getValue("OSC:ENVIARFLOWINPOSITION", true);
    
    etiquetaFromRegion = XML.getValue("OSC:ETIQUETA:FROMREGION", "/frombflow/flowInRegion");
    
    etiquetaFromPosition = XML.getValue("OSC:ETIQUETA:FROMPOSITION", "/frombflow/flowInPosition");
    
    //---------------- CAM --------------------
    deviceID = XML.getValue("CAM:DEVICEID", 0);
    hMirror = XML.getValue("CAM:HMIRROR", false);
    vMirror = XML.getValue("CAM:VMIRROR", false);
    
    paso = XML.getValue("CAM:WARPING:PASO", 5);
    
    warp[0].x = XML.getValue("CAM:WARPING:AX", 0);
    warp[0].y = XML.getValue("CAM:WARPING:Ay", 0);
    warp[1].x = XML.getValue("CAM:WARPING:BX", camWidth);
    warp[1].y = XML.getValue("CAM:WARPING:BY", 0);
    warp[2].x = XML.getValue("CAM:WARPING:CX", camWidth);
    warp[2].y = XML.getValue("CAM:WARPING:CY", camHeight);
    warp[3].x = XML.getValue("CAM:WARPING:DX", 0);
    warp[3].y = XML.getValue("CAM:WARPING:DY", camHeight);
    
    //---------------- FLOW --------------------
    fbPyrScale = XML.getValue("FLOW:PYRSCALE", 0.5);
    fbPolySigma = XML.getValue("FLOW:POLYSIGMA", 1.5);
    
    fbLevels = XML.getValue("FLOW:LEVELS", 4);
    fbIterations = XML.getValue("FLOW:ITERATIONS", 2);
    fbPolyN = XML.getValue("FLOW:POLYN", 7);
    fbWinSize = XML.getValue("FLOW:WINSIZE", 32);
    
    fbUseGaussian = XML.getValue("FLOW:GAUSSIAN", false);
    
    ofLog(OF_LOG_NOTICE,xmlMessage);
}
//--------------------------------------------------------------
void ofApp::saveSettings(){
    //XML.setValue("BACKGROUND:COLOR:RED", red);
    XML.clear();

    //---------------- OSC --------------------
    XML.setValue("OSC:PUERTOOUT", puertoOUT);
    XML.setValue("OSC:PUERTOIN", puertoIN);
    XML.setValue("OSC:HOST", host);
    XML.setValue("OSC:ETIQUETA:TOTALFLOW", etiquetaTotalFlow);
    XML.setValue("OSC:ENVIARTOTALFLOW", enviarTotalFlow);
    
    XML.setValue("OSC:ETIQUETA:AVERAGEFLOW", etiquetaAverageFlow);
    XML.setValue("OSC:ENVIARAVERAGEFLOW", enviarAverageFlow);
    
    XML.setValue("OSC:ETIQUETA:TOTALFLOWINREGION", etiquetaTotalFlowInRegion);
    XML.setValue("OSC:ENVIARTOTALFLOWINREGION", enviarTotalFlowInRegion);
    
    XML.setValue("OSC:ETIQUETA:AVERAGEFLOWINREGION", etiquetaAverageFlowInRegion);
    XML.setValue("OSC:ENVIARAVERAGEFLOWINREGION", enviarAverageFlowInRegion);
    
    XML.setValue("OSC:ETIQUETA:TOREGION", etiquetaToRegion);
    XML.setValue("OSC:ENVIARFLOWINREGION", enviarFlowInRegion);
    
    XML.setValue("OSC:ETIQUETA:TOPOSITION", etiquetaToPosition);
    XML.setValue("OSC:ENVIARFLOWINPOSITION", enviarFlowInPosition);
    
    XML.setValue("OSC:ETIQUETA:FROMREGION", etiquetaFromRegion);
    
    XML.setValue("OSC:ETIQUETA:FROMPOSITION", etiquetaFromPosition);

    //---------------- CAM --------------------
    XML.setValue("CAM:DEVICEID", deviceID);
    XML.setValue("CAM:HMIRROR", hMirror);
    XML.setValue("CAM:VMIRROR", vMirror);
    
    XML.setValue("CAM:WARPING:PASO", paso);
    XML.setValue("CAM:WARPING:AX", warp[0].x);
    XML.setValue("CAM:WARPING:Ay", warp[0].y);
    XML.setValue("CAM:WARPING:BX", warp[1].x);
    XML.setValue("CAM:WARPING:BY", warp[1].y);
    XML.setValue("CAM:WARPING:CX", warp[2].x);
    XML.setValue("CAM:WARPING:CY", warp[2].y);
    XML.setValue("CAM:WARPING:DX", warp[3].x);
    XML.setValue("CAM:WARPING:DY", warp[3].y);

    //---------------- FLOW --------------------
    XML.setValue("FLOW:PYRSCALE", fbPyrScale);
    XML.setValue("FLOW:POLYSIGMA", fbPolySigma);
    XML.setValue("FLOW:LEVELS", fbLevels);
    XML.setValue("FLOW:ITERATIONS", fbIterations);
    XML.setValue("FLOW:POLYN", fbPolyN);
    XML.setValue("FLOW:WINSIZE", fbWinSize);
    XML.setValue("FLOW:GAUSSIAN", fbUseGaussian);
        
    XML.saveFile("mySettings.xml");
    xmlMessage ="settings saved to xml!";
    ofLog(OF_LOG_NOTICE,xmlMessage);
}

//--------------------------------------------------------------
void ofApp::keyPressed(ofKeyEventArgs& e){
    
    #if __APPLE__
    if(e.key == 's' && e.hasModifier(OF_KEY_COMMAND)){
        saveSettings();
    }
    #else
    if(e.key == 19 ){
        saveSettings();
    }
    
    #endif
    else if(e.key == 'd'){
        deleteAreas = !deleteAreas;
    }
    else if(e.key == '1'){
        cualPunto = 0;
    }
    else if(e.key == '2'){
        cualPunto = 1;
    }
    else if(e.key == '3'){
        cualPunto = 2;
    }
    else if(e.key == '4'){
        cualPunto = 3;
    }
    else if(e.key == 'w'){
        warpON = !warpON;
    }
    else if(e.key == OF_KEY_LEFT && warpON){
        warp[cualPunto].x -= paso;
        warp[cualPunto].x = ofClamp(warp[cualPunto].x, 0, camWidth);
    }
    else if(e.key == OF_KEY_RIGHT && warpON){
        warp[cualPunto].x += paso;
        warp[cualPunto].x = ofClamp(warp[cualPunto].x, 0, camWidth);
    }
    else if(e.key == OF_KEY_UP && warpON){
        warp[cualPunto].y -= paso;
        warp[cualPunto].y = ofClamp(warp[cualPunto].y, 0, camHeight);
    }
    else if(e.key == OF_KEY_DOWN && warpON){
        warp[cualPunto].y += paso;
        warp[cualPunto].y = ofClamp(warp[cualPunto].y, 0, camHeight);
    }
}
//--------------------------------------------------------------
void ofApp::keyReleased(ofKeyEventArgs& e){
}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button){
    if(!deleteAreas && !warpON ){
        float mx = ofMap(x, 0, ofGetWidth(), 0, flow.getWidth());
        float my = ofMap(y, 0, ofGetHeight(), 0, flow.getHeight());
        ofVec2f p2(mx,my);
        rect.set(p1,p2.x-p1.x,p2.y-p1.y);
        drawRect = true;
    }
    if(moverPunto){
        if(x >= 0 && x<= ofGetWidth() && y>=0 && y <=ofGetHeight()){
            warp[cualPunto].x = ofMap(x, 0, ofGetWidth(), 0, camWidth);
            warp[cualPunto].y = ofMap(y, 0, ofGetHeight(), 0, camHeight);
        }
    }
}
//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button){
    
    if(warpON){
        for(int i=0; i<4; i++){
            if(corner[i].inside(x, y)){
                cualPunto = i;
                moverPunto = true;
                break;
            }
        }
    }else{
        if(!deleteAreas){
            float mx = ofMap(x, 0, ofGetWidth(), 0, flow.getWidth());
            float my = ofMap(y, 0, ofGetHeight(), 0, flow.getHeight());
            p1.set(mx,my);
        }
    }
}
//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button){
    drawRect = false;
    if(!deleteAreas && !warpON){
        float mx = ofClamp(ofMap(x, 0, ofGetWidth(), 0, flow.getWidth()),0, flow.getWidth() );
        float my = ofClamp(ofMap(y, 0, ofGetHeight(), 0, flow.getHeight()), 0, flow.getHeight());
        
        ofVec2f p2(mx,my);
        rect.set(p1,p2.x-p1.x,p2.y-p1.y);
    
        if(rect.getWidth() > MIN_SIZE && rect.getHeight() > MIN_SIZE){
            ofRectangle r;
            r = rect;
            areas.push_back(r);
        }
    }
    moverPunto = false;
}

//--------------------------------------------------------------
void ofApp::warpingReset(){
    
    if(resetWarping){
        A = ofPoint(0, 0);
        B = ofPoint(camWidth, 0);
        C = ofPoint(camWidth, camHeight);
        D = ofPoint(0, camHeight);
        
        warp[0] = A;
        warp[1] = B;
        warp[2] = C;
        warp[3] = D;
        resetWarping = false;
    }
}


