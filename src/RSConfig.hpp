/**
  Special solution to read bed configurations for slicing from Repetier-Server

*/
#ifndef RSConfig_hpp
#define RSConfig_hpp

#include "rapidjson/document.h"
#include "libslic3r/Format/RMF.hpp"
#include "libslic3r/Model.hpp"

namespace Slic3r {
class CLI;

class TriangleStringData {
    
};



class RSConfig {
    
    static void importModels(const rapidjson::Document::Array &list, CLI &cli);
    static void applyGlobalConfigs(const rapidjson::Document::Object &obj, CLI &cli);
    static void read_from_file(Model &model, const std::string& input_file, DynamicPrintConfig* config, ConfigSubstitutionContext* config_substitutions, Model::LoadAttributes options);
public:
    static bool loadRSConfig(std::string config, CLI &cli);
    
    static std::string configDirectory;
};
}

#endif // RSConfig_hpp
