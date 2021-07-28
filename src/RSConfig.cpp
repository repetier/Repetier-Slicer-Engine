#include "PrusaSlicer.hpp"

#include <fstream>
#include <iostream>
#include <boost/nowide/iostream.hpp>
#include <boost/nowide/fstream.hpp>
#include <boost/filesystem.hpp>

namespace Slic3r {

std::string RSConfig::configDirectory;

bool RSConfig::loadRSConfig(std::string configFile, CLI &cli) {
    boost::nowide::cout << "Reading Repetier-Server config file " << configFile << std::endl;
    try {
        boost::nowide::ifstream t(configFile);
        std::string str;
        
        t.seekg(0, std::ios::end);
        str.reserve(t.tellg());
        t.seekg(0, std::ios::beg);
        
        str.assign((std::istreambuf_iterator<char>(t)),
                   std::istreambuf_iterator<char>());
        
        rapidjson::Document doc;
        doc.ParseInsitu((char*)str.c_str());
        if(!doc.IsObject()) {
            return false;
        }
        configDirectory = boost::filesystem::path(configFile).parent_path().string();
        
        // Load models
        if(doc.HasMember("models") && doc["models"].IsArray()) {
            importModels(doc["models"].GetArray(), cli);
        }
        
        // Apply global configuration
        if(doc.HasMember("global_config") && doc["global_config"].IsObject()) {
            applyGlobalConfigs(doc["global_config"].GetObj(), cli);
        }
        return true;
    }
    catch (...) {
        return false;
    }
}

void RSConfig::applyGlobalConfigs(const rapidjson::Document::Object &obj, CLI &cli) {
    DynamicPrintConfig  config;
    const ForwardCompatibilitySubstitutionRule   config_substitution_rule = cli.m_config.option<ConfigOptionEnum<ForwardCompatibilitySubstitutionRule>>("config_compatibility", true)->value;
    ConfigSubstitutionContext substitutions_ctxt(config_substitution_rule);
    for(auto &i: obj) {
        try {
            if(i.value.IsString()) {
                std::string name = i.name.GetString();
                std::string value = i.value.GetString();
                config.set_deserialize(name, value,  substitutions_ctxt);
            }
        }
        catch (UnknownOptionException & /* e */) {
            // ignore
        }
    }
    if (! substitutions_ctxt.substitutions.empty()) {
        boost::nowide::cout << "The following configuration values were substituted\n";
        for (const ConfigSubstitution &subst : substitutions_ctxt.substitutions)
            boost::nowide::cout << "\tkey = \"" << subst.opt_def->opt_key << "\"\t loaded = \"" << subst.old_value << "\tsubstituted = \"" << subst.new_value->serialize() << "\"\n";
    }
    config.normalize_fdm();
    cli.m_print_config.apply(config);
}
void RSConfig::importModels(const rapidjson::Document::Array &list, CLI &cli) {
    const ForwardCompatibilitySubstitutionRule   config_substitution_rule = cli.m_config.option<ConfigOptionEnum<ForwardCompatibilitySubstitutionRule>>("config_compatibility", true)->value;
    PrinterTechnology printer_technology = Slic3r::printer_technology(cli.m_config);
    for(auto &m: list) {
        Model model;
        std::string file;
        try {
        const rapidjson::Document::Object &o = m.GetObject();
        if(!o.HasMember("file") || !o.HasMember("parts")) {
            continue; // each model needs a file and parts
        }
        file = (boost::filesystem::path(configDirectory) / o["file"].GetString()).string();
        // When loading an AMF or 3MF, config is imported as well, including the printer technology.
        DynamicPrintConfig config;
        ConfigSubstitutionContext config_substitutions(config_substitution_rule);
        //FIXME should we check the version here? // | Model::LoadAttribute::CheckVersion ?
        model = Model::read_from_file(file, &config, &config_substitutions, Model::LoadAttribute::AddDefaultInstances);
        PrinterTechnology other_printer_technology = Slic3r::printer_technology(config);
        if (printer_technology == ptUnknown) {
            printer_technology = other_printer_technology;
        }
        else if (printer_technology != other_printer_technology && other_printer_technology != ptUnknown) {
            boost::nowide::cerr << "Mixing configurations for FFF and SLA technologies" << std::endl;
            return;
        }
        if (! config_substitutions.substitutions.empty()) {
            boost::nowide::cout << "The following configuration values were substituted when loading \" << file << \":\n";
            for (const ConfigSubstitution& subst : config_substitutions.substitutions)
                boost::nowide::cout << "\tkey = \"" << subst.opt_def->opt_key << "\"\t loaded = \"" << subst.old_value << "\tsubstituted = \"" << subst.new_value->serialize() << "\"\n";
        }
        // config is applied to m_print_config before the current m_config values.
        config += std::move(cli.m_print_config);
        cli.m_print_config = std::move(config);
        }
        catch (std::exception& e) {
            boost::nowide::cerr << file << ": " << e.what() << std::endl;
            return;
        }
        if (model.objects.empty()) {
            boost::nowide::cerr << "Error: file is empty: " << file << std::endl;
            continue;
        }
        cli.m_models.push_back(model);
    }
}
}
