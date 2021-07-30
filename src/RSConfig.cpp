#include "PrusaSlicer.hpp"
#include "libslic3r/Format/AMF.hpp"
#include "libslic3r/Format/OBJ.hpp"
#include "libslic3r/Format/PRUS.hpp"
#include "libslic3r/Format/STL.hpp"
#include "libslic3r/Format/3mf.hpp"
#include "libslic3r/Format/RMF.hpp"

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
    RMFPartData::parts.clear();
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
    Model model;
    for(auto &m: list) {
        std::string file;
        try {
            const rapidjson::Document::Object &o = m.GetObject();
            if(!o.HasMember("file") || !o.HasMember("parts")) {
                continue; // each model needs a file and parts
            }
            file = (boost::filesystem::path(configDirectory) / o["file"].GetString()).string();
            auto partsFile = o["parts"].GetArray();
            RMFPartData::parts.clear();
            for(auto &p: partsFile) {
                const auto &po = p.GetObj();
                RMFPartData pData;
                /* {
                 "id": 1,
                 "volume_type": "ModelPart",
                 "seam": [],
                 "support": [],
                 "layer_heights": "",
                 "transform": "1 0 0 0 1 0 0 0 1 125 105 15",
                 "config": {}
               }*/
                if(po.HasMember("volume_type")) {
                    pData.volume_type = ModelVolume::type_from_string(po["volume_type"].GetString());
                } else {
                    pData.volume_type = ModelVolumeType::MODEL_PART;
                }
                if(po.HasMember("layer_heights")) {
                    pData.layer_heights = po["layer_heights"].GetString();
                }
                if(po.HasMember("transform")) {
                    pData.transform = po["transform"].GetString();
                } else {
                    pData.transform = "1 0 0 0 1 0 0 0 1 0 0 0";
                }
                if(po.HasMember("config")) {
                    for(auto &i: po["config"].GetObj()) {
                        if(i.value.IsString()) {
                            std::string name = i.name.GetString();
                            std::string value = i.value.GetString();
                            (*pData.config)["name"] = value;
                        }
                    }
                }
                if(po.HasMember("seam") && po["seam"].IsArray()) {
                    for(auto &i: po["seam"].GetArray()) {
                        const auto &obj = i.GetObj();
                        (*pData.seam)[obj["i"].GetInt()] = obj["d"].GetString();
                    }
                }
                if(po.HasMember("support") && po["support"].IsArray()) {
                    for(auto &i: po["support"].GetArray()) {
                        const auto &obj = i.GetObj();
                        (*pData.support)[obj["i"].GetInt()] = obj["d"].GetString();
                    }
                }
                if(po.HasMember("ranges") && po["ranges"].IsArray()) {
                    for(auto &i: po["ranges"].GetArray()) {
                        const auto &obj = i.GetObj();
                        RMFRange range;
                        range.z_min = (obj.HasMember("z_min") ? obj["z_min"].GetFloat() : 0);
                        range.z_max = (obj.HasMember("z_max") ? obj["z_max"].GetFloat() : range.z_min);
                        if(obj.HasMember("config")) {
                            for(auto &i: obj["config"].GetObj()) {
                                if(i.value.IsString()) {
                                    std::string name = i.name.GetString();
                                    std::string value = i.value.GetString();
                                    (*range.config)[name] = value;
                                }
                            }
                        }
                        pData.ranges->emplace_back(range);
                    }
                }
                if(po.HasMember("volumes") && po["volumes"].IsArray()) {
                    for(auto &i: po["volumes"].GetArray()) {
                        const auto &obj = i.GetObj();
                        RMFVolume vol;
                        vol.start = (obj.HasMember("start") ? obj["start"].GetInt() : 0);
                        vol.end = (obj.HasMember("end") ? obj["end"].GetInt() : vol.start);
                        vol.name = (obj.HasMember("name") ? obj["name"].GetString() : "");
                        if(obj.HasMember("config")) {
                            for(auto &i: obj["config"].GetObj()) {
                                if(i.value.IsString()) {
                                    std::string name = i.name.GetString();
                                    std::string value = i.value.GetString();
                                    (*vol.config)[name] = value;
                                }
                            }
                        }

                        (*pData.volumes).emplace_back(vol);
                    }

                }
                if(po.HasMember("clones") && po["clones"].IsArray()) {
                    for(auto &i: po["clones"].GetArray()) {
                        const auto &obj = i.GetObj();
                        RMFInstance inst;
                        inst.transform = (obj.HasMember("transform") ? obj["transform"].GetString() : "1 0 0 0 1 0 0 0 1 0 0 0");
                        pData.instances.emplace_back(inst);
                    }

                }

                RMFPartData::parts.push_back(pData);
            }
            // When loading an AMF or 3MF, config is imported as well, including the printer technology.
            DynamicPrintConfig config;
            ConfigSubstitutionContext config_substitutions(config_substitution_rule);
            //FIXME should we check the version here? // | Model::LoadAttribute::CheckVersion ?
            read_from_file(model, file, &config, &config_substitutions, Model::LoadAttribute::AddDefaultInstances);
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
    }
    RMFPartData::parts.clear();
    cli.m_models.push_back(model);
}
// Loading model from a file, it may be a simple geometry file as STL or OBJ, however it may be a project file as well.
void RSConfig::read_from_file(Model &model, const std::string& input_file, DynamicPrintConfig* config, ConfigSubstitutionContext* config_substitutions, Model::LoadAttributes options)
{
    DynamicPrintConfig temp_config;
    ConfigSubstitutionContext temp_config_substitutions_context(ForwardCompatibilitySubstitutionRule::EnableSilent);
    if (config == nullptr)
        config = &temp_config;
    if (config_substitutions == nullptr)
        config_substitutions = &temp_config_substitutions_context;

    bool result = false;
    if (boost::algorithm::iends_with(input_file, ".stl"))
        result = load_stl(input_file.c_str(), &model);
    else if (boost::algorithm::iends_with(input_file, ".obj"))
        result = load_obj(input_file.c_str(), &model);
    else if (boost::algorithm::iends_with(input_file, ".amf") || boost::algorithm::iends_with(input_file, ".amf.xml"))
        result = load_amf(input_file.c_str(), config, config_substitutions, &model, options & Model::LoadAttribute::CheckVersion);
    else if (boost::algorithm::iends_with(input_file, ".3mf"))
        //FIXME options & LoadAttribute::CheckVersion ?
        result = load_3mf(input_file.c_str(), *config, *config_substitutions, &model, false);
    else if (boost::algorithm::iends_with(input_file, ".rmf"))
        result = load_rmf(input_file.c_str(), *config, *config_substitutions, &model, false);
    else if (boost::algorithm::iends_with(input_file, ".prusa"))
        result = load_prus(input_file.c_str(), &model);
    else
        throw Slic3r::RuntimeError("Unknown file format. Input file must have .stl, .obj, .amf(.xml) or .prusa extension.");

    if (! result)
        throw Slic3r::RuntimeError("Loading of a model file failed.");

    if (model.objects.empty())
        throw Slic3r::RuntimeError("The supplied file couldn't be read because it's empty");
    
    for (ModelObject *o : model.objects)
        o->input_file = input_file;
    
    if (options & Model::LoadAttribute::AddDefaultInstances)
        model.add_default_instances();

    CustomGCode::update_custom_gcode_per_print_z_from_config(model.custom_gcode_per_print_z, config);
    CustomGCode::check_mode_for_custom_gcode_per_print_z(model.custom_gcode_per_print_z);

    sort_remove_duplicates(config_substitutions->substitutions);
}

}
