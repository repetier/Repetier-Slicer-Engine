/**
  Special solution to read bed configurations for slicing from Repetier-Server

*/
#ifndef RSConfig_hpp
#define RSConfig_hpp

#include "rapidjson/document.h"

namespace Slic3r {
class CLI;
class RSConfig {
    
    static void importModels(const rapidjson::Document::Array &list, CLI &cli);
    static void applyGlobalConfigs(const rapidjson::Document::Object &obj, CLI &cli);
public:
    static bool loadRSConfig(std::string config, CLI &cli);
    
    static std::string configDirectory;
};
}

#endif // RSConfig_hpp
