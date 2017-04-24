#include <opendavinci/odcore/io/StringListener.h>

#define ParseSensors_H_

namespace automotive {
    namespace miniature {

        // This class will handle the bytes received via a serial link.

        class ParseSensors : public odcore::io::StringListener {

            // Your class needs to implement the method void nextString(const std::string &s).
            virtual void nextString(const std::string &s);
            
            virtual void SendString(const char[]);
            };
    }
}
