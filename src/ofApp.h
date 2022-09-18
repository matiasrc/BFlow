#pragma once

#include "ofMain.h"
#include "ofxCv.h"
#include "ofxOpenCv.h"
#include "ofxImGui.h"
#include "ofxOsc.h"
#include "ofxXmlSettings.h"

class ofApp : public ofBaseApp {
public:
    void setup();
    void update();
    void draw();
    void keyPressed(ofKeyEventArgs& e);
    void keyReleased(ofKeyEventArgs& e);
    void setupCam(int devID);
    void resetCameraSettings(int devID);
    void mousePressed(int x, int y, int button);
    void mouseReleased(int x, int y, int button);
    void mouseDragged(int x, int y, int button);

	
    //----------------- CAM -------------------
    ofVideoGrabber          cam;
    ofImage                 camPixels;
    vector<ofVideoDevice>   wdevices;
    vector<string>          devicesVector;
    vector<int>             devicesID;
    float                   camWidth, camHeight;
    int                     imagePixelNumber;
    string                  deviceName;
    int                     deviceID;
    bool                    needReset;
    bool                    isOneDeviceAvailable;
    bool                    hMirror, vMirror;
    
    //----------------- FLOW -------------------
    ofxCv::FlowFarneback    flow;
    
    float fbPyrScale, fbPolySigma;
    int fbLevels, fbIterations, fbPolyN, fbWinSize;
    bool fbUseGaussian;

    ofVec2f p1;
    ofRectangle rect;
    bool drawRect;
    vector <ofRectangle> areas;
    
    bool deleteAreas;
    int MIN_SIZE;
    
    //----------------- GUI -------------------
    void drawGui();
    ofxImGui::Gui gui;
            
    //----------------- OSC -------------------
    // out
    ofxOscSender sender;
    int puertoIN;
    string host;
    string etiquetaTotalFlow;
    string etiquetaAverageFlow;
    string etiquetaTotalFlowInRegion;
    string etiquetaAverageFlowInRegion;
    string etiquetaFromRegion;
    string etiquetaFromPosition;
    bool enviarTotalFlow;
    bool enviarAverageFlow;
    bool enviarTotalFlowInRegion;
    bool enviarAverageFlowInRegion;
    bool enviarFlowInRegion;
    bool enviarFlowInPosition;
    
    string etiquetaToRegion;
    string etiquetaToPosition;
    
    
    /*
    /totalFlow
    /averageFlow
    /totalFlowInRegion
    /averageFlowInRegion
    /fromBflow/FlowInRegion
    /fromBflow/flowInPosition
     */
    
    void enviarOsc(string etiqueda, float valor);
    void enviarOsc(string etiqueda, vector<float> valores);
    
    // in
    int puertoOUT;
    ofxOscReceiver receiver;
    vector<ofxOscMessage> inMessages;
    void recibirOsc();
    
    //----------------- XML -------------------
    ofxXmlSettings XML;
    void saveSettings();
    void loadSettings();
    string xmlMessage;    
};
