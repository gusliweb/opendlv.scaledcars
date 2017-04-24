/**
 * proxy - Sample application to encapsulate HW/SW interfacing with embedded systems.
 * Copyright (C) 2012 - 2015 Christian Berger
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include <ctype.h>
#include <cstring>
#include <cmath>
#include <iostream>

#include "opendavinci/odcore/base/KeyValueConfiguration.h"
#include "opendavinci/odcore/data/Container.h"
#include "opendavinci/odcore/data/TimeStamp.h"
#include <stdint.h>
#include <string>
#include <memory>
#include <opendavinci/odcore/wrapper/SerialPort.h>
#include <opendavinci/odcore/wrapper/SerialPortFactory.h>
#include <ctype.h>
#include <cstring>
#include <cmath>
#include <iostream>
#include "automotivedata/GeneratedHeaders_AutomotiveData.h"
#include "opendavinci/GeneratedHeaders_OpenDaVINCI.h"

using namespace std;
#include <opendavinci/odcore/base/Thread.h>
// We add some of OpenDaVINCI's namespaces for the sake of readability.
using namespace odcore;
using namespace odcore::wrapper;
#include "OpenCVCamera.h"

#ifdef HAVE_UEYE
#include "uEyeCamera.h"
#endif
#include "ParseSensors.h"

#include "Proxy.h"

namespace automotive {
    namespace miniature {

        using namespace std;
        using namespace odcore::base;
        using namespace odcore::data;
        using namespace odtools::recorder;
        string SERIAL_PORT;
        uint32_t BAUD_RATE;
        char myStr[33];
        bool add = false;
        unsigned int i = 0;
        Proxy::Proxy(const int32_t &argc, char **argv) :
            TimeTriggeredConferenceClientModule(argc, argv, "proxy"),
            m_recorder(),
            m_camera()
        {}

        Proxy::~Proxy() {
        }

        void Proxy::setUp() {
            // This method will be call automatically _before_ running body().
            if (getFrequency() < 20) {
                cerr << endl << endl << "Proxy: WARNING! Running proxy with a LOW frequency (consequence: data updates are too seldom and will influence your algorithms in a negative manner!) --> suggestions: --freq=20 or higher! Current frequency: " << getFrequency() << " Hz." << endl << endl << endl;
            }

            // Get configuration data.
            KeyValueConfiguration kv = getKeyValueConfiguration();

            // Create built-in recorder.
            const bool useRecorder = kv.getValue<uint32_t>("proxy.useRecorder") == 1;
            if (useRecorder) {
                // URL for storing containers.
                stringstream recordingURL;
                recordingURL << "file://" << "proxy_" << TimeStamp().getYYYYMMDD_HHMMSS_noBlankNoColons() << ".rec";
                // Size of memory segments.
                const uint32_t MEMORY_SEGMENT_SIZE = getKeyValueConfiguration().getValue<uint32_t>("global.buffer.memorySegmentSize");
                // Number of memory segments.
                const uint32_t NUMBER_OF_SEGMENTS = getKeyValueConfiguration().getValue<uint32_t>("global.buffer.numberOfMemorySegments");
                // Run recorder in asynchronous mode to allow real-time recording in background.
                const bool THREADING = true;
                // Dump shared images and shared data?
                const bool DUMP_SHARED_DATA = getKeyValueConfiguration().getValue<uint32_t>("proxy.recorder.dumpshareddata") == 1;

                m_recorder = unique_ptr<Recorder>(new Recorder(recordingURL.str(), MEMORY_SEGMENT_SIZE, NUMBER_OF_SEGMENTS, THREADING, DUMP_SHARED_DATA));
            }

            // Create the camera grabber.
            const string NAME = getKeyValueConfiguration().getValue<string>("proxy.camera.name");
            string TYPE = getKeyValueConfiguration().getValue<string>("proxy.camera.type");
            std::transform(TYPE.begin(), TYPE.end(), TYPE.begin(), ::tolower);
            const uint32_t ID = getKeyValueConfiguration().getValue<uint32_t>("proxy.camera.id");
            const uint32_t WIDTH = getKeyValueConfiguration().getValue<uint32_t>("proxy.camera.width");
            const uint32_t HEIGHT = getKeyValueConfiguration().getValue<uint32_t>("proxy.camera.height");
            const uint32_t BPP = getKeyValueConfiguration().getValue<uint32_t>("proxy.camera.bpp");
            SERIAL_PORT = getKeyValueConfiguration().getValue<string>("proxy.actuator.serialport");
            BAUD_RATE = getKeyValueConfiguration().getValue<uint32_t>("proxy.sensor.serialspeed");
            const bool DEBUG = getKeyValueConfiguration().getValue< bool >("proxy.camera.debug") == 1;
            const bool FLIPPED = getKeyValueConfiguration().getValue< uint32_t >("proxy.camera.flipped") == 1;

            if (TYPE.compare("opencv") == 0) {
                m_camera = unique_ptr< Camera >(new OpenCVCamera(NAME, ID, WIDTH, HEIGHT, BPP, DEBUG, FLIPPED));
            }
            if (TYPE.compare("ueye") == 0) {
#ifdef HAVE_UEYE
                m_camera = unique_ptr<Camera>(new uEyeCamera(NAME, ID, WIDTH, HEIGHT, BPP));
#endif
            }


            if (m_camera.get() == NULL) {
                cerr << "No valid camera type defined." << endl;
            }

        }

        void Proxy::tearDown() {
            // This method will be call automatically _after_ return from body().
        }

        void Proxy::distribute(Container c) {
            // Store data to recorder.
            if (m_recorder.get() != NULL) {
                // Time stamp data before storing.
                c.setReceivedTimeStamp(TimeStamp());
                m_recorder->store(c);
            }

            // Share data.
            getConference().send(c);
        }
        void ParseSensors::nextString(const string &s) {
            cerr << "Received " << s.length() << " bytes containing '" << s << "'" << endl;
            /*char* chr = strdup(s.c_str());
              for(unsigned int p = 0; p < s.length(); p++){
              if(s.length() > 11){
              break;
              }
              if(chr[i] == "["){
              add = true;
              }
              if(chr[i] == "]" && add){ 
              cerr << " READY STRING " << myStr << endl;
              */
            SendString(myStr);
            i = 0;/*
                     add = false;
                     }
                     if(add){
                     myStr[i] = chr[p];   
                     i++;
                     }
                     }*/
        }

        void ParseSensors::SendString(const char chararr[]){
            cerr  << chararr[0] << " senddd" << endl;
            /* cerr << " ready send " << sendvalues << endl;
               SensorBoardData sbd(sensors, sendvalues); // Pack and send SBD
               Container csbd(sbd);
               getConference().send(csbd);
               VehicleData vehicleData; // Pack and send VD
               vehicleData.setAbsTraveledPath((handler.getDist()));
               Container cvd(vd);
               getConference().send(cvd);*/
        }


        // This method will do the main data processing job.
        odcore::data::dmcp::ModuleExitCodeMessage::ModuleExitCode Proxy::body() {

            std::shared_ptr<SerialPort> serial(SerialPortFactory::createSerialPort(SERIAL_PORT, BAUD_RATE));
            ParseSensors handler;
            serial->setStringListener(&handler);
            const uint32_t ONE_SECOND = 1000 * 1000;
            odcore::base::Thread::usleepFor(ONE_SECOND);

            // Start receiving bytes.
            serial->start();
            // Stop receiving bytes and unregister our handler.
            //serial->stop();



            uint32_t captureCounter = 0;
            while (getModuleStateAndWaitForRemainingTimeInTimeslice() == odcore::data::dmcp::ModuleStateMessage::RUNNING) {
                // Capture frame.
                if (m_camera.get() != NULL) {
                    odcore::data::image::SharedImage si = m_camera->capture();

                    Container c(si);
                    distribute(c);
                    captureCounter++;
                }

                /*try {
                  cerr << "SERIAL " << SERIAL_PORT << " BAUD " << BAUD_RATE << endl;

                  serial->send("Hello World\r\n");
                  }
                  catch(string &exception) {
                  cerr << "Serial port could not be created: " << exception << endl;
                  }
                  const uint32_t ONE_SECOND = 1000 * 1000;
                  odcore::base::Thread::usleepFor(ONE_SECOND);
                  */
                Container vehicleControlContainer = getKeyValueDataStore().get(automotive::VehicleControl::ID());
                VehicleControl vehicleControlData = vehicleControlContainer.getData<VehicleControl> ();

                //double speed = vehicleControlData.getSpeed(); 
                double Angle = vehicleControlData.getSteeringWheelAngle();
                // int speed1 = (int) speed;

                int Angle1 = (int) 90 + (Angle * (180 / 3.1415926535)); // Cast angle to int
                std::string angleString = std::to_string(Angle1);
                /*if(speed1 < 0){ 
                  serial->send("B," + angleString + "]");
                  } 

                  if(speed1 == 0)){*/
                //   serial->send("S," + angleString + "]");
                /*} 
                */
                // else{
                // negative or positive angles
                if(Angle1 > -1){

                serial->send("[F+" + angleString + "]");
                }
                else{
                    serial->send("[F" + angleString + "]");
                }
                //}
            }

            cout << "Proxy: Captured " << captureCounter << " frames." << endl;

            serial->stop();
            serial->setStringListener(NULL);

            return odcore::data::dmcp::ModuleExitCodeMessage::OKAY;
        }

    }
} // automotive::miniature

