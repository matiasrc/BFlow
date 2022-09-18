

#include "ofApp.h"

// Helper to display a little (?) mark which shows a tooltip when hovered.
// In your own code you may want to display an actual icon if you are using a merged icon fonts (see docs/FONTS.txt)
static void HelpMarker(const char* desc)
{
    ImGui::TextDisabled("(?)");
    if (ImGui::IsItemHovered())
    {
        ImGui::BeginTooltip();
        ImGui::PushTextWrapPos(ImGui::GetFontSize() * 35.0f);
        ImGui::TextUnformatted(desc);
        ImGui::PopTextWrapPos();
        ImGui::EndTooltip();
    }
}

//--------------------------------------------------------------
void ofApp::drawGui(){
    //required to call this at beginning
    gui.begin();
    // -------- MENU PRINCIPAL --------
    if (ImGui::BeginMainMenuBar())
    {
        if (ImGui::BeginMenu("|Flow|"))
        {
            //----------------- FLOW -------------------
            ImGui::Checkbox("Usar Gaussian", &fbUseGaussian);
            ImGui::Separator();
        
            ImGui::SliderFloat("Scale", &fbPyrScale, 0.0f, 0.99f); ImGui::SameLine(); HelpMarker("Scale");
            ImGui::SliderFloat("PolySigma", &fbPolySigma, 1.1f, 2.0f); ImGui::SameLine(); HelpMarker("Poly Sigma");
            
            ImGui::SliderInt("Levels", &fbLevels, 1, 8); ImGui::SameLine(); HelpMarker("Levels");
            ImGui::SliderInt("Iterations", &fbIterations, 1, 8); ImGui::SameLine(); HelpMarker("Iterations");
            ImGui::SliderInt("Poly Number", &fbPolyN, 5, 10); ImGui::SameLine(); HelpMarker("Poly Number");
            ImGui::SliderInt("Window Size", &fbWinSize, 4, 64); ImGui::SameLine(); HelpMarker("Window Size");
        
            ImGui::Separator();
            ImGui::Checkbox("Edidar Areas", &deleteAreas);
            
            ImGui::EndMenu();
        }
        
        if (ImGui::BeginMenu("|Entrada|"))
            {
                static const char* item_current = devicesVector[deviceID].c_str();
                if(ImGui::BeginCombo(" ", item_current)){
        
                    for(int i=0; i < devicesVector.size(); ++i){
                        const bool isSelected = (deviceID == i);
                        if(ImGui::Selectable(devicesVector[i].c_str(), isSelected)){
                            deviceID = i;
                            resetCameraSettings(deviceID);
                            item_current = devicesVector[i].c_str();
                        }
                        if(isSelected){
                            ImGui::SetItemDefaultFocus();
                        }
                    }
                ImGui::EndCombo();
                }
                ImGui::SameLine(); HelpMarker("Elegir el dispositivo de entrada");
                
                ImGui::Separator();
                ImGui::Checkbox("ESPEJAR HORIZONTAL", &hMirror);
                ImGui::Checkbox("ESPEJAR VERTICAL", &vMirror);
                        
            ImGui::EndMenu();

        }
        
        if (ImGui::BeginMenu("|OSC|"))
        {
            if(ImGui::InputInt("port de salida", &puertoOUT)) sender.setup(host, puertoOUT);
            ImGui::SameLine(); HelpMarker("puerto de SALIDA");
            
            static char str1[128];
            strcpy(str1, host.c_str());
            if( ImGui::InputTextWithHint("ip", "enter ip address here",str1, IM_ARRAYSIZE(str1))){
                host = ofVAArgsToString(str1);
                sender.setup(host, puertoOUT);
                //ofLogVerbose() << "--------CAMBIO DE HOST: " << host;
            }
            ImGui::SameLine(); HelpMarker("dirección ip del receptor de mensajes");
            
            ImGui::Separator();
            
            static char totalFlowaddress[128];
            strcpy(totalFlowaddress, etiquetaTotalFlow.c_str());
            if( ImGui::InputTextWithHint("a1", "tipear etiqueta TOTAL FLOW",totalFlowaddress, IM_ARRAYSIZE(totalFlowaddress))){
                etiquetaTotalFlow = ofVAArgsToString(totalFlowaddress);
            }
            ImGui::SameLine(); HelpMarker("etiqueta (debe comenzar con /) ");
            
            ImGui::SameLine();ImGui::Checkbox("ENVIAR TF", &enviarTotalFlow);
            ImGui::SameLine(); HelpMarker("habilitar / deshabilitar el envío");
            
            static char averageFlowaddress[128];
            strcpy(averageFlowaddress, etiquetaAverageFlow.c_str());
            if( ImGui::InputTextWithHint("a2", "tipear etiqueta AVERAGE FLOW",averageFlowaddress, IM_ARRAYSIZE(averageFlowaddress))){
                etiquetaAverageFlow = ofVAArgsToString(averageFlowaddress);
            }
            ImGui::SameLine(); HelpMarker("etiqueta (debe comenzar con /) ");
            
            ImGui::SameLine();ImGui::Checkbox("ENVIAR AF", &enviarAverageFlow);
            ImGui::SameLine(); HelpMarker("habilitar / deshabilitar el envío");
            
            static char totalFlowInRegionaddress[128];
            strcpy(totalFlowInRegionaddress, etiquetaTotalFlowInRegion.c_str());
            if( ImGui::InputTextWithHint("a3", "tipear etiqueta TOTAL FLOW IN REGION",totalFlowInRegionaddress, IM_ARRAYSIZE(totalFlowInRegionaddress))){
                etiquetaTotalFlowInRegion = ofVAArgsToString(totalFlowInRegionaddress);
            }
            ImGui::SameLine(); HelpMarker("etiqueta (debe comenzar con /) ");
            
            ImGui::SameLine();ImGui::Checkbox("ENVIAR TFIR", &enviarTotalFlowInRegion);
            ImGui::SameLine(); HelpMarker("habilitar / deshabilitar el envío");
            
            static char averageFlowInRegionaddress[128];
            strcpy(averageFlowInRegionaddress, etiquetaAverageFlowInRegion.c_str());
            if( ImGui::InputTextWithHint("a4", "tipear etiqueta AVERAGE FLOW IN REGION",averageFlowInRegionaddress, IM_ARRAYSIZE(averageFlowInRegionaddress))){
                etiquetaAverageFlowInRegion = ofVAArgsToString(averageFlowInRegionaddress);
            }
            ImGui::SameLine(); HelpMarker("etiqueta (debe comenzar con /) ");
            
            ImGui::SameLine();ImGui::Checkbox("ENVIAR AFIR", &enviarAverageFlowInRegion);
            ImGui::SameLine(); HelpMarker("habilitar / deshabilitar el envío");
            
            //-----------------
            ImGui::Separator();ImGui::Separator();
            ImGui::Text("RESPUESTA A PEDIDO");
            
           
            ImGui::Text("Entrada");
            
            if(ImGui::InputInt("port de entrada", &puertoIN)) receiver.setup(puertoIN);
            ImGui::SameLine(); HelpMarker("puerto de ENTRADA");
            
            static char toRegionaddress[128];
            strcpy(toRegionaddress, etiquetaToRegion.c_str());
            if( ImGui::InputTextWithHint("a5", "tipear etiqueta FLOW IN REGION",toRegionaddress, IM_ARRAYSIZE(toRegionaddress))){
                etiquetaToRegion = ofVAArgsToString(toRegionaddress);
            }
            ImGui::SameLine(); HelpMarker("etiqueta (debe comenzar con /) ");
            
            static char toPositionaddress[128];
            strcpy(toPositionaddress, etiquetaToPosition.c_str());
            if( ImGui::InputTextWithHint("a6", "tipear etiqueta FLOW IN REGION",toPositionaddress, IM_ARRAYSIZE(toPositionaddress))){
                etiquetaToPosition = ofVAArgsToString(toPositionaddress);
            }
            ImGui::SameLine(); HelpMarker("etiqueta (debe comenzar con /) ");
            
            
            
            ImGui::Separator();
            ImGui::Text("Salida");
            
            static char fromRegionaddres[128];
            strcpy(fromRegionaddres, etiquetaFromRegion.c_str());
            if( ImGui::InputTextWithHint("a7", "tipear etiqueta FLOW IN REGION",fromRegionaddres, IM_ARRAYSIZE(fromRegionaddres))){
                etiquetaFromRegion = ofVAArgsToString(fromRegionaddres);
            }
            ImGui::SameLine(); HelpMarker("etiqueta (debe comenzar con /) ");
            
            ImGui::SameLine();ImGui::Checkbox("ENVIAR FIR", &enviarFlowInRegion);
            ImGui::SameLine(); HelpMarker("habilitar / deshabilitar el envío");
            
            static char fromPositionaddress[128];
            strcpy(fromPositionaddress, etiquetaFromPosition.c_str());
            if( ImGui::InputTextWithHint("a8", "tipear etiqueta FLOW IN POSITION",fromPositionaddress, IM_ARRAYSIZE(fromPositionaddress))){
                etiquetaFromPosition = ofVAArgsToString(fromPositionaddress);
            }
            ImGui::SameLine(); HelpMarker("etiqueta (debe comenzar con /) ");
            
            ImGui::SameLine();ImGui::Checkbox("ENVIAR FIP", &enviarFlowInPosition);
            ImGui::SameLine(); HelpMarker("habilitar / deshabilitar el envío");
        
            ImGui::EndMenu();
        }
        if (ImGui::BeginMenu("|Acerca|"))
        {
            ImGui::Text("BFlow");
            ImGui::Separator();
            ImGui::Text("Software experimental para captura de movimiento.");
            ImGui::Text("utilizando las técnicas de optical flow, ");
            ImGui::Text("Esta aplicación está en desarrollo y no tiene soporte");
            ImGui::Text("..............");
            ImGui::Text("Desarrollado por Matías Romero Costas (Biopus)");
            ImGui::Text("www.biopus.ar");

            ImGui::EndMenu();
        }
        /*
        if (ImGui::BeginMenu("| Guardar", "cmd+s"))
        {
            saveSettings();
            ofLogVerbose() << "Configuración guardada";
            if (ImGui::MenuItem("Guardar", "cmd+s")) {
                saveSettings();
                ofLogVerbose() << "Configuración guardada";
            }
            
            ImGui::EndMenu();
        }
        */
        ImGui::EndMainMenuBar();
    }
    // Menú flotante
    gui.end();
}
