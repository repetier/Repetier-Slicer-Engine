#include "../libslic3r.h"
#include "../Exception.hpp"
#include "../Model.hpp"
#include "../Utils.hpp"
#include "../GCode.hpp"
#include "../Geometry.hpp"
#include "../GCode/ThumbnailData.hpp"
#include "../Time.hpp"

#include "../I18N.hpp"

// #include "3mf.hpp"
#include "RMF.hpp"

#include <limits>
#include <stdexcept>

#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/predicate.hpp>
#include <boost/algorithm/string/replace.hpp>
#include <boost/filesystem/operations.hpp>
#include <boost/nowide/fstream.hpp>
#include <boost/nowide/cstdio.hpp>

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/foreach.hpp>
namespace pt = boost::property_tree;

#include <expat.h>
#include <Eigen/Dense>
#include "miniz_extension.hpp"

Slic3r::Transform3d get_transform_from_rmf_specs_string(const std::string& mat_str)
{
    // check: https://3mf.io/3d-manufacturing-format/ or https://github.com/3MFConsortium/spec_core/blob/master/3MF%20Core%20Specification.md
    // to see how matrices are stored inside 3mf according to specifications
    Slic3r::Transform3d ret = Slic3r::Transform3d::Identity();
    
    if (mat_str.empty())
        // empty string means default identity matrix
        return ret;
    
    std::vector<std::string> mat_elements_str;
    boost::split(mat_elements_str, mat_str, boost::is_any_of(" "), boost::token_compress_on);
    
    unsigned int size = (unsigned int)mat_elements_str.size();
    if (size != 12)
        // invalid data, return identity matrix
        return ret;
    
    unsigned int i = 0;
    // matrices are stored into 3mf files as 4x3
    // we need to transpose them
    for (unsigned int c = 0; c < 4; ++c)
    {
        for (unsigned int r = 0; r < 3; ++r)
        {
            ret(r, c) = ::atof(mat_elements_str[i++].c_str());
        }
    }
    return ret;
}
/*
static float get_unit_factor(const std::string& unit)
{
    const char* text = unit.c_str();
    
    if (::strcmp(text, "micron") == 0)
        return 0.001f;
    else if (::strcmp(text, "centimeter") == 0)
        return 10.0f;
    else if (::strcmp(text, "inch") == 0)
        return 25.4f;
    else if (::strcmp(text, "foot") == 0)
        return 304.8f;
    else if (::strcmp(text, "meter") == 0)
        return 1000.0f;
    else
        // default "millimeters" (see specification)
        return 1.0f;
}
*/
namespace Slic3r {

std::vector<RMFPartData> RMFPartData::parts;

RMFVolume::RMFVolume() {
    config = new std::map<std::string, std::string>();
    start = end = 0;
}

RMFVolume::RMFVolume(RMFVolume const& other) {
    end = other.end;
    start = other.start;
    name = other.name;
    config = new std::map<std::string, std::string>(*other.config);
}

RMFVolume::RMFVolume(RMFVolume&& that) noexcept {
    config = nullptr;
    swap(that);
}

RMFVolume::~RMFVolume() {
    delete config;
}

RMFVolume &RMFVolume::operator=(RMFVolume const& other) {
    RMFVolume copy(other);
    this->swap(copy);
    return *this;
}

RMFVolume& RMFVolume::operator=(RMFVolume&& that) noexcept {
    swap(that);
    return *this;
}

void RMFVolume::swap(RMFVolume& rhs) noexcept {
    std::swap(start, rhs.start);
    std::swap(end, rhs.end);
    std::swap(name, rhs.name);
    std::swap(config, rhs.config);
}

// --------------------------------

RMFRange::RMFRange() {
    config = new std::map<std::string,std::string>();
    objectId = -1;
}

RMFRange::RMFRange(RMFRange const& other) {
    z_min = other.z_min;
    z_max = other.z_max;
    objectId = other.objectId;
    config = new std::map<std::string,std::string>(*other.config);
}

RMFRange::RMFRange(RMFRange&& that) noexcept {
    config = nullptr;
    swap(that);
}

RMFRange::~RMFRange() {
    delete config;
}

RMFRange &RMFRange::operator=(RMFRange const& other) {
    RMFRange copy(other);
    this->swap(copy);
    return *this;
}

RMFRange& RMFRange::operator=(RMFRange&& that) noexcept {
    swap(that);
    return *this;
}

void RMFRange::swap(RMFRange& rhs) noexcept {
    std::swap(z_min, rhs.z_min);
    std::swap(z_max, rhs.z_max);
    std::swap(objectId, rhs.objectId);
    std::swap(config, rhs.config);
}

// --------------------------------

RMFPartData::RMFPartData() {
    config = new std::map<std::string,std::string>();
    seam = new std::map<int, std::string>();
    support = new std::map<int, std::string>();
    ranges = new std::vector<RMFRange>();
    volumes = new std::vector<RMFVolume>();
}

RMFPartData::RMFPartData(RMFPartData const& other) {
    volume_type = other.volume_type;
    layer_heights = other.layer_heights;
    transform = other.transform;
    name = other.name;
    instances = other.instances;
    config = new std::map<std::string,std::string>(*other.config);
    seam = new std::map<int, std::string>(*other.seam);
    support = new std::map<int, std::string>(*other.support);
    ranges = new std::vector<RMFRange>(*other.ranges);
    volumes = new std::vector<RMFVolume>(*other.volumes);
}

RMFPartData::RMFPartData(RMFPartData&& that) noexcept {
    config = nullptr;
    seam = nullptr;
    support = nullptr;
    ranges = nullptr;
    volumes = nullptr;
    swap(that);
}

RMFPartData::~RMFPartData() {
    delete config;
    delete seam;
    delete support;
    delete ranges;
    delete volumes;
}

RMFPartData &RMFPartData::operator=(RMFPartData const& other) {
    RMFPartData copy(other);
    this->swap(copy);
    return *this;
}

RMFPartData& RMFPartData::operator=(RMFPartData&& that) noexcept {
    swap(that);
    return *this;
}

void RMFPartData::swap(RMFPartData& rhs) noexcept {
    std::swap(config, rhs.config);
    std::swap(seam, rhs.seam);
    std::swap(support, rhs.support);
    std::swap(ranges, rhs.ranges);
    std::swap(volumes, rhs.volumes);
    std::swap(instances, rhs.instances);
    std::swap(layer_heights, rhs.layer_heights);
    std::swap(volume_type, rhs.volume_type);
    std::swap(transform, rhs.transform);
    std::swap(name, rhs.name);
}

RMFOutStream::RMFOutStream() {
    used = 0;
    reserved = 1024 * 5000;
    data = new char[reserved];
}

RMFOutStream::~RMFOutStream() {
    delete[] data;
}

void RMFOutStream::write(const char *ptr, size_t sz) {
    if(used + sz > reserved) {
        reserved = reserved + sz + 1024 * 10000;
        char *newData = new char[reserved];
        memcpy(newData, data, used);
        delete[] data;
        data = newData;
    }
    memcpy(data + used, ptr, sz);
    used += sz;
}

void RMFOutStream::computeBitsForMax(uint32_t m) {
    bits = 1;
    uint32_t v = 1;
    while(m > v && bits < 32) {
        bits++;
        v = (v << 1) | 1;
    }
    bitMask = v;
}

void RMFOutStream::startBlock(uint32_t _id) {
    used = 0;
    id = _id;
    bits = 32;
    bitsWritten = 0;
    bitBuffer = 0;
}

void RMFOutStream::writeBits(uint32_t val) {
    val &= bitMask;
    if (bitsWritten + bits >= 32) { // full 32 bit so write them
        bitBuffer = (bitBuffer << (32 - bitsWritten)) | (val >> (bits - (32 - bitsWritten)));
        writeUInt32(bitBuffer);
        bitBuffer = val;
        bitsWritten = (bits - (32 - bitsWritten));
    } else {
        bitBuffer = (bitBuffer << bits) | val;
        bitsWritten += bits;
    }
}
void RMFOutStream::writeBlock(std::ofstream &out) {
    if(bitsWritten != 0) {
        bitBuffer <<= 32 - bitsWritten;
        writeUInt32(bitBuffer);
    }
    length = static_cast<uint32_t>(used);
    out.write((char*)&id, sizeof(uint32_t));
    out.write((char*)&length, sizeof(uint32_t));
    out.write(data, used);
    used = 0;
}

// ------------ RMFInStream --------------

void RMFInStream::readString(std::string &s) {
    char c;
    s = "";
    do {
        in.get(c);
        pos++;
        if(c) {
            s += c;
        }
    } while(c != 0);
}

bool RMFInStream::connect(std::string filename) {
    in.open(filename, std::ios::in|std::ios::binary);
    char buffer[5];
    in.read(buffer, 4);
    if(buffer[0] != '@' || buffer[1] != 'r' || buffer[2] != 'm' || buffer[3] != 'f')
        return false;
    readUInt32(version);
    id = 0;
    length = 0;
    pos = 0;
    return true;
}

void RMFInStream::toEndOfBlock() {
    if(length != pos) {
        in.seekg(/* static_cast<uint32_t>(in.tellg()) + */ length - pos, std::ios_base::cur);
    }
}

void RMFInStream::scanBlockType() {
    readUInt32(id);
    readUInt32(length);
    pos = 0;
    bitsLeft = 0;
}

static uint32_t bitfix[33] = {
    0,0x1,0x3,0x7,0xf,0x1f,0x3f,0x7f,0xff,
    0x1ff,0x3ff,0x7ff,0xfff,0x1fff,0x3fff,0x7fff,0xffff,
    0x1ffff,0x3ffff,0x7ffff,0xfffff,0x1fffff,0x3fffff,0x7fffff,0xffffff,
    0x1ffffff,0x3ffffff,0x7ffffff,0xfffffff,
    0x1fffffff,0x3fffffff,0x7fffffff,0xffffffff
};

void RMFInStream::readBits(uint32_t &x) {
    if(bitsLeft >= bits) {
        x = (bitBuffer >> (bitsLeft - bits)) & bitfix[bits];
        bitsLeft -= bits;
        return;
    }
    int bitsRequired = bits - bitsLeft;
    x = (bitBuffer & bitfix[bitsLeft]) << bitsRequired;
    readUInt32(bitBuffer);
    bitsLeft = 32 - bitsRequired;
    x = x | ((bitBuffer >> bitsLeft) & bitfix[bitsRequired]);
}




//! macro used to mark string used at localization,
//! return same string
#define L(s) (s)
#define _(s) Slic3r::I18N::translate(s)

// Base class with error messages management
class _RMF_Base
{
    std::vector<std::string> m_errors;
    
protected:
    void add_error(const std::string& error) { m_errors.push_back(error); }
    void clear_errors() { m_errors.clear(); }
    
public:
    void log_errors()
    {
        for (const std::string& error : m_errors)
        {
            printf("%s\n", error.c_str());
        }
    }
};

class _RMF_Importer : public _RMF_Base
{
    struct Geometry
    {
        std::vector<float> vertices;
        std::vector<unsigned int> triangles;
        std::vector<std::string> custom_supports;
        std::vector<std::string> custom_seam;
        
        bool empty()
        {
            return vertices.empty() || triangles.empty();
        }
        
        void reset()
        {
            vertices.clear();
            triangles.clear();
            custom_supports.clear();
            custom_seam.clear();
        }
    };
    
    struct CurrentObject
    {
        Geometry geometry;
        ModelObject* object;
        
        CurrentObject()
        {
            reset();
        }
        
        void reset()
        {
            geometry.reset();
            object = nullptr;
        }
    };
    
    Model* m_model;
    RMFPartData *part;
    Geometry geometry;
    ModelObject* object;
    float m_unit_factor;
    std::string m_name;
    
public:
    _RMF_Importer();
    ~_RMF_Importer();
    
    bool load_model_from_file(const std::string& filename, Model& model, DynamicPrintConfig& config, ConfigSubstitutionContext& config_substitutions, bool check_version);
    
private:
    bool _load_model_from_file(const std::string& filename, Model& model, DynamicPrintConfig& config, ConfigSubstitutionContext& config_substitutions);
    
    void applyPartData(ConfigSubstitutionContext& config_substitutions);
    void importVolumes(ConfigSubstitutionContext& config_substitutions);
    static std::vector<coordf_t> splitCommaSepCoords(const std::string &heights);
};

_RMF_Importer::_RMF_Importer()
: m_model(nullptr)
, part(nullptr)
, m_unit_factor(1.0f)
, m_name("")
{
}

_RMF_Importer::~_RMF_Importer()
{
}

bool _RMF_Importer::load_model_from_file(const std::string& filename, Model& model, DynamicPrintConfig& config, ConfigSubstitutionContext& config_substitutions, bool check_version)
{
    m_model = &model;
    part = nullptr;
    m_unit_factor = 1.0f;
    geometry.reset();
    object = nullptr;
    clear_errors();
    
    return _load_model_from_file(filename, model, config, config_substitutions);
}

bool _RMF_Importer::_load_model_from_file(const std::string& filename, Model& model, DynamicPrintConfig& config, ConfigSubstitutionContext& config_substitutions)
{
    // rmf
    
    RMFInStream in;
    TriangleMesh mesh;
    std::string modelName = boost::filesystem::basename(filename);
    if(!in.connect(filename)) {
        add_error("Unable to open the file " + filename);
        return false;
    }
    m_name = boost::filesystem::path(filename).filename().stem().string();
    uint32_t nVertices, nTriangles;
    float x,y,z;
    int objIdx = -1;
    while(in.in.good()) {
        in.scanBlockType();
        if(!in.in.good())
            break;
        switch(in.id) {
            case 1: // Metadata global
            {
                uint32_t nMetaData, i;
                in.readUInt32(nMetaData);
                for(i = 0 ; i < nMetaData; i++) {
                    std::string key, value;
                    in.readString(key);
                    in.readString(value);
                }
            }
                break;
            case 2: // new model, metadata
            {
                objIdx++;
                if(RMFPartData::parts.size() > objIdx) {
                    part = &RMFPartData::parts[objIdx];
                } else {
                    part = nullptr;
                }
                uint32_t nMetaData, i;
                // reset current data
                geometry.reset();
                // create new object (it may be removed later if no instances are generated from it)
                object = m_model->add_object();
                if (object == nullptr)
                {
                    add_error("Unable to create object");
                    return false;
                }
                in.readUInt32(nMetaData);
                for(i = 0 ; i < nMetaData; i++) {
                    std::string key, value;
                    in.readString(key);
                    in.readString(value);
                    if(key == "name") {
                        modelName = value;
                        object->name = value;
                    }
                }
                if (object->name.empty())
                    object->name = m_name + "_" + std::to_string(m_model->objects.size());
            }
                break;
            case 3: // points
                nVertices = in.length / 12;
                for(uint32_t i = 0; i < nVertices; i++) {
                    in.readFloat(x);
                    in.readFloat(y);
                    in.readFloat(z);
                    geometry.vertices.push_back(m_unit_factor * x);
                    geometry.vertices.push_back(m_unit_factor * y);
                    geometry.vertices.push_back(m_unit_factor * z);
                }
                break;
            case 4: // triangles
                    // reset current triangles
                in.readUInt32(in.bits);
                in.readUInt32(nTriangles);
                for(uint32_t i = 0; i < nTriangles; i++) {
                    uint32_t a,b,c;
                    in.readBits(a);
                    in.readBits(b);
                    in.readBits(c);
                    geometry.triangles.push_back((unsigned int)a);
                    geometry.triangles.push_back((unsigned int)b);
                    geometry.triangles.push_back((unsigned int)c);
                    
                    if(part) {
                        auto sup = part->support->find(i);
                        geometry.custom_supports.push_back(sup == part->support->end() ? std::string() :  sup->second);
                        auto seam = part->seam->find(i);
                        geometry.custom_seam.push_back(seam == part->seam->end() ? std::string() : seam->second);
                    } else {
                        geometry.custom_supports.push_back(std::string());
                        geometry.custom_seam.push_back(std::string());
                    }
                }
                
                // apply all object dependend assignments here
                
                if(part) {
                    applyPartData(config_substitutions);
                }
                // import geometry
                importVolumes(config_substitutions);
                break;
            default:
                break;
        }
        in.toEndOfBlock();
    }
    
    //        // fixes the min z of the model if negative
    //        model.adjust_min_z();
    
    return true;
}

void _RMF_Importer::applyPartData(ConfigSubstitutionContext& config_substitutions) {
    object->name = part->name;
    // Per object configuration
    for(const auto &i: *part->config) {
        object->config.set_deserialize(i.first, i.second, config_substitutions);
    }
    // Layer heights
    if(part->layer_heights.length() > 0) {
        object->layer_height_profile.set(_RMF_Importer::splitCommaSepCoords(part->layer_heights));
    }
    
    // Assign ranges
    t_layer_config_ranges config_ranges;
    for(auto &i: *part->ranges) {
        DynamicPrintConfig config;
        for(auto &j: *i.config) {
            std::string key = j.first;
            config.set_deserialize(key, j.second, config_substitutions);
        }
        config_ranges[{ i.z_min, i.z_max }].assign_config(std::move(config));
    }
    object->layer_config_ranges = std::move(config_ranges);
    
    // skip sla support points
    // skip sla drain holes
    
}

void _RMF_Importer::importVolumes(ConfigSubstitutionContext& config_substitutions) {
    std::vector<RMFVolume> volumes;
    uint32_t max_triangle = geometry.triangles.size() / 3;
    if(part) {
        for(auto &i: *part->volumes) {
            if(i.end >= max_triangle) {
                i.end = max_triangle - 1;
            }
            volumes.emplace_back(i);
        }
    }
    if(volumes.size() == 0) {
        RMFVolume v;
        v.start = 0;
        v.end = max_triangle - 1;
        volumes.emplace_back(v);
    }
    for(auto &volumeDef: volumes) {
        if ((max_triangle <= volumeDef.start) || (volumeDef.end < volumeDef.start))
        {
            add_error("Found invalid triangle id");
            return;
        }
        
        Transform3d volume_matrix_to_object = Transform3d::Identity();
        bool has_transform = false;
        
        // extract the volume transformation from the volume's metadata, if present
        for(auto &i: *volumeDef.config) {
            if(i.first == "matrix") {
                volume_matrix_to_object = Slic3r::Geometry::transform3d_from_string(i.second);
                has_transform             = ! volume_matrix_to_object.isApprox(Transform3d::Identity(), 1e-10);
                break;
            }
        }
        
        // splits volume out of imported geometry
        TriangleMesh triangle_mesh;
        stl_file    &stl             = triangle_mesh.stl;
        unsigned int triangles_count = volumeDef.end - volumeDef.start + 1;
        stl.stats.type = inmemory;
        stl.stats.number_of_facets = (uint32_t)triangles_count;
        stl.stats.original_num_facets = (int)stl.stats.number_of_facets;
        stl_allocate(&stl);
        
        unsigned int src_start_id = volumeDef.start * 3;
        for (unsigned int i = 0; i < triangles_count; ++i)
        {
            unsigned int ii = i * 3;
            stl_facet& facet = stl.facet_start[i];
            for (unsigned int v = 0; v < 3; ++v)
            {
                unsigned int tri_id = geometry.triangles[src_start_id + ii + v] * 3;
                if (tri_id + 2 >= geometry.vertices.size()) {
                    add_error("Malformed triangle mesh");
                    return;
                }
                facet.vertex[v] = Vec3f(geometry.vertices[tri_id + 0], geometry.vertices[tri_id + 1], geometry.vertices[tri_id + 2]);
            }
        }
        
        stl_get_size(&stl);
        triangle_mesh.repair();
        
        ModelVolume* volume = object->add_volume(std::move(triangle_mesh));
        // stores the volume matrix taken from the metadata, if present
        if (has_transform) {
            volume->source.transform = Slic3r::Geometry::Transformation(volume_matrix_to_object);
        }
        volume->calculate_convex_hull();
        
        // recreate custom supports and seam from previously loaded attribute
        for (unsigned i = 0; i < triangles_count; ++i) {
            size_t index = src_start_id/3 + i;
            assert(index < geometry.custom_supports.size());
            assert(index < geometry.custom_seam.size());
            if (! geometry.custom_supports[index].empty()) {
                volume->supported_facets.set_triangle_from_string(i, geometry.custom_supports[index]);
            }
            if (! geometry.custom_seam[index].empty()) {
                volume->seam_facets.set_triangle_from_string(i, geometry.custom_seam[index]);
            }
        }
        
        for(auto &i: *volumeDef.config) {
            if (i.first == "NAME_KEY") {
                volume->name = i.second;
            } else if ((i.first == "modifier") && (i.second == "1")) {
                volume->set_type(ModelVolumeType::PARAMETER_MODIFIER);
            } else if (i.first == "volume_type") {
                volume->set_type(ModelVolume::type_from_string(i.second));
            } else if (i.first == "source_file") {
                volume->source.input_file = i.second;
            } else if (i.first == "source_object_id") {
                volume->source.object_idx = ::atoi(i.second.c_str());
            } else if (i.first == "source_volume_id") {
                volume->source.volume_idx = ::atoi(i.second.c_str());
            } else if (i.first == "source_offset_x") {
                volume->source.mesh_offset(0) = ::atof(i.second.c_str());
            } else if (i.first == "source_offset_y") {
                volume->source.mesh_offset(1) = ::atof(i.second.c_str());
            } else if (i.first == "source_offset_z") {
                volume->source.mesh_offset(2) = ::atof(i.second.c_str());
            } else if (i.first == "source_in_inches") {
                volume->source.is_converted_from_inches = i.second == "1";
            } else {
                volume->config.set_deserialize(i.first, i.second, config_substitutions);
            }
        }
    }
    // add ModelInstances
    
    // first add the original object
    
    ModelInstance *orig = object->add_instance();
    if(part) {
        Transform3d trans = get_transform_from_rmf_specs_string(part->transform);
        Slic3r::Geometry::Transformation t(trans);
        orig->set_transformation(t);
    }
        
    if(part) { // add clones
        for(auto &i: part->instances) {
            orig = object->add_instance();
            Transform3d trans = get_transform_from_rmf_specs_string(i.transform);
            Slic3r::Geometry::Transformation t(trans);
            orig->set_transformation(t);
        }
    }
}

std::vector<coordf_t> _RMF_Importer::splitCommaSepCoords(const std::string &heightsString) {
    std::vector<coordf_t> heights;
    std::vector<std::string> args;
    boost::split(args, heightsString, boost::is_any_of(","));
    for(auto &i: args) {
        heights.push_back((coordf_t)std::atof(i.c_str()));
    }
    return heights;
}

#if false
class _3MF_Exporter : public _3MF_Base
{
    struct BuildItem
    {
        unsigned int id;
        Transform3d transform;
        bool printable;
        
        BuildItem(unsigned int id, const Transform3d& transform, const bool printable)
        : id(id)
        , transform(transform)
        , printable(printable)
        {
        }
    };
    
    struct Offsets
    {
        unsigned int first_vertex_id;
        unsigned int first_triangle_id;
        unsigned int last_triangle_id;
        
        Offsets(unsigned int first_vertex_id)
        : first_vertex_id(first_vertex_id)
        , first_triangle_id(-1)
        , last_triangle_id(-1)
        {
        }
    };
    
    typedef std::map<const ModelVolume*, Offsets> VolumeToOffsetsMap;
    
    struct ObjectData
    {
        ModelObject* object;
        VolumeToOffsetsMap volumes_offsets;
        
        explicit ObjectData(ModelObject* object)
        : object(object)
        {
        }
    };
    
    typedef std::vector<BuildItem> BuildItemsList;
    typedef std::map<int, ObjectData> IdToObjectDataMap;
    
    bool m_fullpath_sources{ true };
    
public:
    bool save_model_to_file(const std::string& filename, Model& model, const DynamicPrintConfig* config, bool fullpath_sources, const ThumbnailData* thumbnail_data = nullptr);
    
private:
    bool _save_model_to_file(const std::string& filename, Model& model, const DynamicPrintConfig* config, const ThumbnailData* thumbnail_data);
    bool _add_content_types_file_to_archive(mz_zip_archive& archive);
    bool _add_thumbnail_file_to_archive(mz_zip_archive& archive, const ThumbnailData& thumbnail_data);
    bool _add_relationships_file_to_archive(mz_zip_archive& archive);
    bool _add_model_file_to_archive(const std::string& filename, mz_zip_archive& archive, const Model& model, IdToObjectDataMap& objects_data);
    bool _add_object_to_model_stream(std::stringstream& stream, unsigned int& object_id, ModelObject& object, BuildItemsList& build_items, VolumeToOffsetsMap& volumes_offsets);
    bool _add_mesh_to_object_stream(std::stringstream& stream, ModelObject& object, VolumeToOffsetsMap& volumes_offsets);
    bool _add_build_to_model_stream(std::stringstream& stream, const BuildItemsList& build_items);
    bool _add_layer_height_profile_file_to_archive(mz_zip_archive& archive, Model& model);
    bool _add_layer_config_ranges_file_to_archive(mz_zip_archive& archive, Model& model);
    bool _add_sla_support_points_file_to_archive(mz_zip_archive& archive, Model& model);
    bool _add_sla_drain_holes_file_to_archive(mz_zip_archive& archive, Model& model);
    bool _add_print_config_file_to_archive(mz_zip_archive& archive, const DynamicPrintConfig &config);
    bool _add_model_config_file_to_archive(mz_zip_archive& archive, const Model& model, const IdToObjectDataMap &objects_data);
    bool _add_custom_gcode_per_print_z_file_to_archive(mz_zip_archive& archive, Model& model, const DynamicPrintConfig* config);
};

bool _3MF_Exporter::save_model_to_file(const std::string& filename, Model& model, const DynamicPrintConfig* config, bool fullpath_sources, const ThumbnailData* thumbnail_data)
{
    clear_errors();
    m_fullpath_sources = fullpath_sources;
    return _save_model_to_file(filename, model, config, thumbnail_data);
}

bool _3MF_Exporter::_save_model_to_file(const std::string& filename, Model& model, const DynamicPrintConfig* config, const ThumbnailData* thumbnail_data)
{
    mz_zip_archive archive;
    mz_zip_zero_struct(&archive);
    
    if (!open_zip_writer(&archive, filename)) {
        add_error("Unable to open the file");
        return false;
    }
    
    // Adds content types file ("[Content_Types].xml";).
    // The content of this file is the same for each PrusaSlicer 3mf.
    if (!_add_content_types_file_to_archive(archive))
    {
        close_zip_writer(&archive);
        boost::filesystem::remove(filename);
        return false;
    }
    
    if ((thumbnail_data != nullptr) && thumbnail_data->is_valid())
    {
        // Adds the file Metadata/thumbnail.png.
        if (!_add_thumbnail_file_to_archive(archive, *thumbnail_data))
        {
            close_zip_writer(&archive);
            boost::filesystem::remove(filename);
            return false;
        }
    }
    
    // Adds relationships file ("_rels/.rels").
    // The content of this file is the same for each PrusaSlicer 3mf.
    // The relationshis file contains a reference to the geometry file "3D/3dmodel.model", the name was chosen to be compatible with CURA.
    if (!_add_relationships_file_to_archive(archive))
    {
        close_zip_writer(&archive);
        boost::filesystem::remove(filename);
        return false;
    }
    
    // Adds model file ("3D/3dmodel.model").
    // This is the one and only file that contains all the geometry (vertices and triangles) of all ModelVolumes.
    IdToObjectDataMap objects_data;
    if (!_add_model_file_to_archive(filename, archive, model, objects_data))
    {
        close_zip_writer(&archive);
        boost::filesystem::remove(filename);
        return false;
    }
    
    // Adds layer height profile file ("Metadata/Slic3r_PE_layer_heights_profile.txt").
    // All layer height profiles of all ModelObjects are stored here, indexed by 1 based index of the ModelObject in Model.
    // The index differes from the index of an object ID of an object instance of a 3MF file!
    if (!_add_layer_height_profile_file_to_archive(archive, model))
    {
        close_zip_writer(&archive);
        boost::filesystem::remove(filename);
        return false;
    }
    
    // Adds layer config ranges file ("Metadata/Slic3r_PE_layer_config_ranges.txt").
    // All layer height profiles of all ModelObjects are stored here, indexed by 1 based index of the ModelObject in Model.
    // The index differes from the index of an object ID of an object instance of a 3MF file!
    if (!_add_layer_config_ranges_file_to_archive(archive, model))
    {
        close_zip_writer(&archive);
        boost::filesystem::remove(filename);
        return false;
    }
    
    // Adds sla support points file ("Metadata/Slic3r_PE_sla_support_points.txt").
    // All  sla support points of all ModelObjects are stored here, indexed by 1 based index of the ModelObject in Model.
    // The index differes from the index of an object ID of an object instance of a 3MF file!
    if (!_add_sla_support_points_file_to_archive(archive, model))
    {
        close_zip_writer(&archive);
        boost::filesystem::remove(filename);
        return false;
    }
    
    if (!_add_sla_drain_holes_file_to_archive(archive, model))
    {
        close_zip_writer(&archive);
        boost::filesystem::remove(filename);
        return false;
    }
    
    
    // Adds custom gcode per height file ("Metadata/Prusa_Slicer_custom_gcode_per_print_z.xml").
    // All custom gcode per height of whole Model are stored here
    if (!_add_custom_gcode_per_print_z_file_to_archive(archive, model, config))
    {
        close_zip_writer(&archive);
        boost::filesystem::remove(filename);
        return false;
    }
    
    // Adds slic3r print config file ("Metadata/Slic3r_PE.config").
    // This file contains the content of FullPrintConfing / SLAFullPrintConfig.
    if (config != nullptr)
    {
        if (!_add_print_config_file_to_archive(archive, *config))
        {
            close_zip_writer(&archive);
            boost::filesystem::remove(filename);
            return false;
        }
    }
    
    // Adds slic3r model config file ("Metadata/Slic3r_PE_model.config").
    // This file contains all the attributes of all ModelObjects and their ModelVolumes (names, parameter overrides).
    // As there is just a single Indexed Triangle Set data stored per ModelObject, offsets of volumes into their respective Indexed Triangle Set data
    // is stored here as well.
    if (!_add_model_config_file_to_archive(archive, model, objects_data))
    {
        close_zip_writer(&archive);
        boost::filesystem::remove(filename);
        return false;
    }
    
    if (!mz_zip_writer_finalize_archive(&archive))
    {
        close_zip_writer(&archive);
        boost::filesystem::remove(filename);
        add_error("Unable to finalize the archive");
        return false;
    }
    
    close_zip_writer(&archive);
    
    return true;
}

bool _3MF_Exporter::_add_content_types_file_to_archive(mz_zip_archive& archive)
{
    std::stringstream stream;
    stream << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n";
    stream << "<Types xmlns=\"http://schemas.openxmlformats.org/package/2006/content-types\">\n";
    stream << " <Default Extension=\"rels\" ContentType=\"application/vnd.openxmlformats-package.relationships+xml\" />\n";
    stream << " <Default Extension=\"model\" ContentType=\"application/vnd.ms-package.3dmanufacturing-3dmodel+xml\" />\n";
    stream << " <Default Extension=\"png\" ContentType=\"image/png\" />\n";
    stream << "</Types>";
    
    std::string out = stream.str();
    
    if (!mz_zip_writer_add_mem(&archive, CONTENT_TYPES_FILE.c_str(), (const void*)out.data(), out.length(), MZ_DEFAULT_COMPRESSION))
    {
        add_error("Unable to add content types file to archive");
        return false;
    }
    
    return true;
}

bool _3MF_Exporter::_add_thumbnail_file_to_archive(mz_zip_archive& archive, const ThumbnailData& thumbnail_data)
{
    bool res = false;
    
    size_t png_size = 0;
    void* png_data = tdefl_write_image_to_png_file_in_memory_ex((const void*)thumbnail_data.pixels.data(), thumbnail_data.width, thumbnail_data.height, 4, &png_size, MZ_DEFAULT_LEVEL, 1);
    if (png_data != nullptr)
    {
        res = mz_zip_writer_add_mem(&archive, THUMBNAIL_FILE.c_str(), (const void*)png_data, png_size, MZ_DEFAULT_COMPRESSION);
        mz_free(png_data);
    }
    
    if (!res)
        add_error("Unable to add thumbnail file to archive");
    
    return res;
}

bool _3MF_Exporter::_add_relationships_file_to_archive(mz_zip_archive& archive)
{
    std::stringstream stream;
    stream << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n";
    stream << "<Relationships xmlns=\"http://schemas.openxmlformats.org/package/2006/relationships\">\n";
    stream << " <Relationship Target=\"/" << MODEL_FILE << "\" Id=\"rel-1\" Type=\"http://schemas.microsoft.com/3dmanufacturing/2013/01/3dmodel\" />\n";
    stream << " <Relationship Target=\"/" << THUMBNAIL_FILE << "\" Id=\"rel-2\" Type=\"http://schemas.openxmlformats.org/package/2006/relationships/metadata/thumbnail\" />\n";
    stream << "</Relationships>";
    
    std::string out = stream.str();
    
    if (!mz_zip_writer_add_mem(&archive, RELATIONSHIPS_FILE.c_str(), (const void*)out.data(), out.length(), MZ_DEFAULT_COMPRESSION))
    {
        add_error("Unable to add relationships file to archive");
        return false;
    }
    
    return true;
}

bool _3MF_Exporter::_add_model_file_to_archive(const std::string& filename, mz_zip_archive& archive, const Model& model, IdToObjectDataMap& objects_data)
{
    std::stringstream stream;
    // https://en.cppreference.com/w/cpp/types/numeric_limits/max_digits10
    // Conversion of a floating-point value to text and back is exact as long as at least max_digits10 were used (9 for float, 17 for double).
    // It is guaranteed to produce the same floating-point value, even though the intermediate text representation is not exact.
    // The default value of std::stream precision is 6 digits only!
    stream << std::setprecision(std::numeric_limits<float>::max_digits10);
    stream << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n";
    stream << "<" << MODEL_TAG << " unit=\"millimeter\" xml:lang=\"en-US\" xmlns=\"http://schemas.microsoft.com/3dmanufacturing/core/2015/02\" xmlns:slic3rpe=\"http://schemas.slic3r.org/3mf/2017/06\">\n";
    stream << " <" << METADATA_TAG << " name=\"" << SLIC3RPE_3MF_VERSION << "\">" << VERSION_3MF << "</" << METADATA_TAG << ">\n";
    std::string name = xml_escape(boost::filesystem::path(filename).stem().string());
    stream << " <" << METADATA_TAG << " name=\"Title\">" << name << "</" << METADATA_TAG << ">\n";
    stream << " <" << METADATA_TAG << " name=\"Designer\">" << "</" << METADATA_TAG << ">\n";
    stream << " <" << METADATA_TAG << " name=\"Description\">" << name << "</" << METADATA_TAG << ">\n";
    stream << " <" << METADATA_TAG << " name=\"Copyright\">" << "</" << METADATA_TAG << ">\n";
    stream << " <" << METADATA_TAG << " name=\"LicenseTerms\">" << "</" << METADATA_TAG << ">\n";
    stream << " <" << METADATA_TAG << " name=\"Rating\">" << "</" << METADATA_TAG << ">\n";
    std::string date = Slic3r::Utils::utc_timestamp(Slic3r::Utils::get_current_time_utc());
    // keep only the date part of the string
    date = date.substr(0, 10);
    stream << " <" << METADATA_TAG << " name=\"CreationDate\">" << date << "</" << METADATA_TAG << ">\n";
    stream << " <" << METADATA_TAG << " name=\"ModificationDate\">" << date << "</" << METADATA_TAG << ">\n";
    stream << " <" << METADATA_TAG << " name=\"Application\">" << SLIC3R_APP_KEY << "-" << SLIC3R_VERSION << "</" << METADATA_TAG << ">\n";
    stream << " <" << RESOURCES_TAG << ">\n";
    
    // Instance transformations, indexed by the 3MF object ID (which is a linear serialization of all instances of all ModelObjects).
    BuildItemsList build_items;
    
    // The object_id here is a one based identifier of the first instance of a ModelObject in the 3MF file, where
    // all the object instances of all ModelObjects are stored and indexed in a 1 based linear fashion.
    // Therefore the list of object_ids here may not be continuous.
    unsigned int object_id = 1;
    for (ModelObject* obj : model.objects)
    {
        if (obj == nullptr)
            continue;
        
        // Index of an object in the 3MF file corresponding to the 1st instance of a ModelObject.
        unsigned int curr_id = object_id;
        IdToObjectDataMap::iterator object_it = objects_data.insert(IdToObjectDataMap::value_type(curr_id, ObjectData(obj))).first;
        // Store geometry of all ModelVolumes contained in a single ModelObject into a single 3MF indexed triangle set object.
        // object_it->second.volumes_offsets will contain the offsets of the ModelVolumes in that single indexed triangle set.
        // object_id will be increased to point to the 1st instance of the next ModelObject.
        if (!_add_object_to_model_stream(stream, object_id, *obj, build_items, object_it->second.volumes_offsets))
        {
            add_error("Unable to add object to archive");
            return false;
        }
    }
    
    stream << " </" << RESOURCES_TAG << ">\n";
    
    // Store the transformations of all the ModelInstances of all ModelObjects, indexed in a linear fashion.
    if (!_add_build_to_model_stream(stream, build_items))
    {
        add_error("Unable to add build to archive");
        return false;
    }
    
    stream << "</" << MODEL_TAG << ">\n";
    
    std::string out = stream.str();
    
    if (!mz_zip_writer_add_mem(&archive, MODEL_FILE.c_str(), (const void*)out.data(), out.length(), MZ_DEFAULT_COMPRESSION))
    {
        add_error("Unable to add model file to archive");
        return false;
    }
    
    return true;
}

bool _3MF_Exporter::_add_object_to_model_stream(std::stringstream& stream, unsigned int& object_id, ModelObject& object, BuildItemsList& build_items, VolumeToOffsetsMap& volumes_offsets)
{
    unsigned int id = 0;
    for (const ModelInstance* instance : object.instances)
    {
        assert(instance != nullptr);
        if (instance == nullptr)
            continue;
        
        unsigned int instance_id = object_id + id;
        stream << "  <" << OBJECT_TAG << " id=\"" << instance_id << "\" type=\"model\">\n";
        
        if (id == 0)
        {
            if (!_add_mesh_to_object_stream(stream, object, volumes_offsets))
            {
                add_error("Unable to add mesh to archive");
                return false;
            }
        }
        else
        {
            stream << "   <" << COMPONENTS_TAG << ">\n";
            stream << "    <" << COMPONENT_TAG << " objectid=\"" << object_id << "\" />\n";
            stream << "   </" << COMPONENTS_TAG << ">\n";
        }
        
        Transform3d t = instance->get_matrix();
        // instance_id is just a 1 indexed index in build_items.
        assert(instance_id == build_items.size() + 1);
        build_items.emplace_back(instance_id, t, instance->printable);
        
        stream << "  </" << OBJECT_TAG << ">\n";
        
        ++id;
    }
    
    object_id += id;
    return true;
}

bool _3MF_Exporter::_add_mesh_to_object_stream(std::stringstream& stream, ModelObject& object, VolumeToOffsetsMap& volumes_offsets)
{
    stream << "   <" << MESH_TAG << ">\n";
    stream << "    <" << VERTICES_TAG << ">\n";
    
    unsigned int vertices_count = 0;
    for (ModelVolume* volume : object.volumes)
    {
        if (volume == nullptr)
            continue;
        
        if (!volume->mesh().repaired)
            throw Slic3r::FileIOError("store_3mf() requires repair()");
        if (!volume->mesh().has_shared_vertices())
            throw Slic3r::FileIOError("store_3mf() requires shared vertices");
        
        volumes_offsets.insert(VolumeToOffsetsMap::value_type(volume, Offsets(vertices_count))).first;
        
        const indexed_triangle_set &its = volume->mesh().its;
        if (its.vertices.empty())
        {
            add_error("Found invalid mesh");
            return false;
        }
        
        vertices_count += (int)its.vertices.size();
        
        const Transform3d& matrix = volume->get_matrix();
        
        for (size_t i = 0; i < its.vertices.size(); ++i)
        {
            stream << "     <" << VERTEX_TAG << " ";
            Vec3f v = (matrix * its.vertices[i].cast<double>()).cast<float>();
            stream << "x=\"" << v(0) << "\" ";
            stream << "y=\"" << v(1) << "\" ";
            stream << "z=\"" << v(2) << "\" />\n";
        }
    }
    
    stream << "    </" << VERTICES_TAG << ">\n";
    stream << "    <" << TRIANGLES_TAG << ">\n";
    
    unsigned int triangles_count = 0;
    for (ModelVolume* volume : object.volumes)
    {
        if (volume == nullptr)
            continue;
        
        VolumeToOffsetsMap::iterator volume_it = volumes_offsets.find(volume);
        assert(volume_it != volumes_offsets.end());
        
        const indexed_triangle_set &its = volume->mesh().its;
        
        // updates triangle offsets
        volume_it->second.first_triangle_id = triangles_count;
        triangles_count += (int)its.indices.size();
        volume_it->second.last_triangle_id = triangles_count - 1;
        
        for (int i = 0; i < int(its.indices.size()); ++ i)
        {
            stream << "     <" << TRIANGLE_TAG << " ";
            for (int j = 0; j < 3; ++j)
            {
                stream << "v" << j + 1 << "=\"" << its.indices[i][j] + volume_it->second.first_vertex_id << "\" ";
            }
            
            std::string custom_supports_data_string = volume->supported_facets.get_triangle_as_string(i);
            if (! custom_supports_data_string.empty())
                stream << CUSTOM_SUPPORTS_ATTR << "=\"" << custom_supports_data_string << "\" ";
            
            std::string custom_seam_data_string = volume->seam_facets.get_triangle_as_string(i);
            if (! custom_seam_data_string.empty())
                stream << CUSTOM_SEAM_ATTR << "=\"" << custom_seam_data_string << "\" ";
            
            stream << "/>\n";
        }
    }
    
    stream << "    </" << TRIANGLES_TAG << ">\n";
    stream << "   </" << MESH_TAG << ">\n";
    
    return true;
}

bool _3MF_Exporter::_add_build_to_model_stream(std::stringstream& stream, const BuildItemsList& build_items)
{
    if (build_items.size() == 0)
    {
        add_error("No build item found");
        return false;
    }
    
    stream << " <" << BUILD_TAG << ">\n";
    
    for (const BuildItem& item : build_items)
    {
        stream << "  <" << ITEM_TAG << " " << OBJECTID_ATTR << "=\"" << item.id << "\" " << TRANSFORM_ATTR << "=\"";
        for (unsigned c = 0; c < 4; ++c)
        {
            for (unsigned r = 0; r < 3; ++r)
            {
                stream << item.transform(r, c);
                if ((r != 2) || (c != 3))
                    stream << " ";
            }
        }
        stream << "\" " << PRINTABLE_ATTR << "=\"" << item.printable << "\" />\n";
    }
    
    stream << " </" << BUILD_TAG << ">\n";
    
    return true;
}

bool _3MF_Exporter::_add_layer_height_profile_file_to_archive(mz_zip_archive& archive, Model& model)
{
    std::string out = "";
    char buffer[1024];
    
    unsigned int count = 0;
    for (const ModelObject* object : model.objects)
    {
        ++count;
        const std::vector<double>& layer_height_profile = object->layer_height_profile.get();
        if ((layer_height_profile.size() >= 4) && ((layer_height_profile.size() % 2) == 0))
        {
            sprintf(buffer, "object_id=%d|", count);
            out += buffer;
            
            // Store the layer height profile as a single semicolon separated list.
            for (size_t i = 0; i < layer_height_profile.size(); ++i)
            {
                sprintf(buffer, (i == 0) ? "%f" : ";%f", layer_height_profile[i]);
                out += buffer;
            }
            
            out += "\n";
        }
    }
    
    if (!out.empty())
    {
        if (!mz_zip_writer_add_mem(&archive, LAYER_HEIGHTS_PROFILE_FILE.c_str(), (const void*)out.data(), out.length(), MZ_DEFAULT_COMPRESSION))
        {
            add_error("Unable to add layer heights profile file to archive");
            return false;
        }
    }
    
    return true;
}

bool _3MF_Exporter::_add_layer_config_ranges_file_to_archive(mz_zip_archive& archive, Model& model)
{
    std::string out = "";
    pt::ptree tree;
    
    unsigned int object_cnt = 0;
    for (const ModelObject* object : model.objects)
    {
        object_cnt++;
        const t_layer_config_ranges& ranges = object->layer_config_ranges;
        if (!ranges.empty())
        {
            pt::ptree& obj_tree = tree.add("objects.object","");
            
            obj_tree.put("<xmlattr>.id", object_cnt);
            
            // Store the layer config ranges.
            for (const auto& range : ranges)
            {
                pt::ptree& range_tree = obj_tree.add("range", "");
                
                // store minX and maxZ
                range_tree.put("<xmlattr>.min_z", range.first.first);
                range_tree.put("<xmlattr>.max_z", range.first.second);
                
                // store range configuration
                const ModelConfig& config = range.second;
                for (const std::string& opt_key : config.keys())
                {
                    pt::ptree& opt_tree = range_tree.add("option", config.opt_serialize(opt_key));
                    opt_tree.put("<xmlattr>.opt_key", opt_key);
                }
            }
        }
    }
    
    if (!tree.empty())
    {
        std::ostringstream oss;
        pt::write_xml(oss, tree);
        out = oss.str();
        
        // Post processing("beautification") of the output string for a better preview
        boost::replace_all(out, "><object",      ">\n <object");
        boost::replace_all(out, "><range",       ">\n  <range");
        boost::replace_all(out, "><option",      ">\n   <option");
        boost::replace_all(out, "></range>",     ">\n  </range>");
        boost::replace_all(out, "></object>",    ">\n </object>");
        // OR just
        boost::replace_all(out, "><",            ">\n<");
    }
    
    if (!out.empty())
    {
        if (!mz_zip_writer_add_mem(&archive, LAYER_CONFIG_RANGES_FILE.c_str(), (const void*)out.data(), out.length(), MZ_DEFAULT_COMPRESSION))
        {
            add_error("Unable to add layer heights profile file to archive");
            return false;
        }
    }
    
    return true;
}

bool _3MF_Exporter::_add_sla_support_points_file_to_archive(mz_zip_archive& archive, Model& model)
{
    std::string out = "";
    char buffer[1024];
    
    unsigned int count = 0;
    for (const ModelObject* object : model.objects)
    {
        ++count;
        const std::vector<sla::SupportPoint>& sla_support_points = object->sla_support_points;
        if (!sla_support_points.empty())
        {
            sprintf(buffer, "object_id=%d|", count);
            out += buffer;
            
            // Store the layer height profile as a single space separated list.
            for (size_t i = 0; i < sla_support_points.size(); ++i)
            {
                sprintf(buffer, (i==0 ? "%f %f %f %f %f" : " %f %f %f %f %f"),  sla_support_points[i].pos(0), sla_support_points[i].pos(1), sla_support_points[i].pos(2), sla_support_points[i].head_front_radius, (float)sla_support_points[i].is_new_island);
                out += buffer;
            }
            out += "\n";
        }
    }
    
    if (!out.empty())
    {
        // Adds version header at the beginning:
        out = std::string("support_points_format_version=") + std::to_string(support_points_format_version) + std::string("\n") + out;
        
        if (!mz_zip_writer_add_mem(&archive, SLA_SUPPORT_POINTS_FILE.c_str(), (const void*)out.data(), out.length(), MZ_DEFAULT_COMPRESSION))
        {
            add_error("Unable to add sla support points file to archive");
            return false;
        }
    }
    return true;
}

bool _3MF_Exporter::_add_sla_drain_holes_file_to_archive(mz_zip_archive& archive, Model& model)
{
    const char *const fmt = "object_id=%d|";
    std::string out;
    
    unsigned int count = 0;
    for (const ModelObject* object : model.objects)
    {
        ++count;
        sla::DrainHoles drain_holes = object->sla_drain_holes;
        
        // The holes were placed 1mm above the mesh in the first implementation.
        // This was a bad idea and the reference point was changed in 2.3 so
        // to be on the mesh exactly. The elevated position is still saved
        // in 3MFs for compatibility reasons.
        for (sla::DrainHole& hole : drain_holes) {
            hole.pos -= hole.normal.normalized();
            hole.height += 1.f;
        }
        
        
        if (!drain_holes.empty())
        {
            out += string_printf(fmt, count);
            
            // Store the layer height profile as a single space separated list.
            for (size_t i = 0; i < drain_holes.size(); ++i)
                out += string_printf((i == 0 ? "%f %f %f %f %f %f %f %f" : " %f %f %f %f %f %f %f %f"),
                                     drain_holes[i].pos(0),
                                     drain_holes[i].pos(1),
                                     drain_holes[i].pos(2),
                                     drain_holes[i].normal(0),
                                     drain_holes[i].normal(1),
                                     drain_holes[i].normal(2),
                                     drain_holes[i].radius,
                                     drain_holes[i].height);
            
            out += "\n";
        }
    }
    
    if (!out.empty())
    {
        // Adds version header at the beginning:
        out = std::string("drain_holes_format_version=") + std::to_string(drain_holes_format_version) + std::string("\n") + out;
        
        if (!mz_zip_writer_add_mem(&archive, SLA_DRAIN_HOLES_FILE.c_str(), static_cast<const void*>(out.data()), out.length(), mz_uint(MZ_DEFAULT_COMPRESSION)))
        {
            add_error("Unable to add sla support points file to archive");
            return false;
        }
    }
    return true;
}

bool _3MF_Exporter::_add_print_config_file_to_archive(mz_zip_archive& archive, const DynamicPrintConfig &config)
{
    char buffer[1024];
    sprintf(buffer, "; %s\n\n", header_slic3r_generated().c_str());
    std::string out = buffer;
    
    for (const std::string &key : config.keys())
        if (key != "compatible_printers")
            out += "; " + key + " = " + config.opt_serialize(key) + "\n";
    
    if (!out.empty())
    {
        if (!mz_zip_writer_add_mem(&archive, PRINT_CONFIG_FILE.c_str(), (const void*)out.data(), out.length(), MZ_DEFAULT_COMPRESSION))
        {
            add_error("Unable to add print config file to archive");
            return false;
        }
    }
    
    return true;
}

bool _3MF_Exporter::_add_model_config_file_to_archive(mz_zip_archive& archive, const Model& model, const IdToObjectDataMap &objects_data)
{
    std::stringstream stream;
    // Store mesh transformation in full precision, as the volumes are stored transformed and they need to be transformed back
    // when loaded as accurately as possible.
    stream << std::setprecision(std::numeric_limits<double>::max_digits10);
    stream << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n";
    stream << "<" << CONFIG_TAG << ">\n";
    
    for (const IdToObjectDataMap::value_type& obj_metadata : objects_data)
    {
        const ModelObject* obj = obj_metadata.second.object;
        if (obj != nullptr)
        {
            // Output of instances count added because of github #3435, currently not used by PrusaSlicer
            stream << " <" << OBJECT_TAG << " " << ID_ATTR << "=\"" << obj_metadata.first << "\" " << INSTANCESCOUNT_ATTR << "=\"" << obj->instances.size() << "\">\n";
            
            // stores object's name
            if (!obj->name.empty())
                stream << "  <" << METADATA_TAG << " " << TYPE_ATTR << "=\"" << OBJECT_TYPE << "\" " << KEY_ATTR << "=\"name\" " << VALUE_ATTR << "=\"" << xml_escape(obj->name) << "\"/>\n";
            
            // stores object's config data
            for (const std::string& key : obj->config.keys())
            {
                stream << "  <" << METADATA_TAG << " " << TYPE_ATTR << "=\"" << OBJECT_TYPE << "\" " << KEY_ATTR << "=\"" << key << "\" " << VALUE_ATTR << "=\"" << obj->config.opt_serialize(key) << "\"/>\n";
            }
            
            for (const ModelVolume* volume : obj_metadata.second.object->volumes)
            {
                if (volume != nullptr)
                {
                    const VolumeToOffsetsMap& offsets = obj_metadata.second.volumes_offsets;
                    VolumeToOffsetsMap::const_iterator it = offsets.find(volume);
                    if (it != offsets.end())
                    {
                        // stores volume's offsets
                        stream << "  <" << VOLUME_TAG << " ";
                        stream << FIRST_TRIANGLE_ID_ATTR << "=\"" << it->second.first_triangle_id << "\" ";
                        stream << LAST_TRIANGLE_ID_ATTR << "=\"" << it->second.last_triangle_id << "\">\n";
                        
                        // stores volume's name
                        if (!volume->name.empty())
                            stream << "   <" << METADATA_TAG << " " << TYPE_ATTR << "=\"" << VOLUME_TYPE << "\" " << KEY_ATTR << "=\"" << NAME_KEY << "\" " << VALUE_ATTR << "=\"" << xml_escape(volume->name) << "\"/>\n";
                        
                        // stores volume's modifier field (legacy, to support old slicers)
                        if (volume->is_modifier())
                            stream << "   <" << METADATA_TAG << " " << TYPE_ATTR << "=\"" << VOLUME_TYPE << "\" " << KEY_ATTR << "=\"" << MODIFIER_KEY << "\" " << VALUE_ATTR << "=\"1\"/>\n";
                        // stores volume's type (overrides the modifier field above)
                        stream << "   <" << METADATA_TAG << " " << TYPE_ATTR << "=\"" << VOLUME_TYPE << "\" " << KEY_ATTR << "=\"" << VOLUME_TYPE_KEY << "\" " <<
                        VALUE_ATTR << "=\"" << ModelVolume::type_to_string(volume->type()) << "\"/>\n";
                        
                        // stores volume's local matrix
                        stream << "   <" << METADATA_TAG << " " << TYPE_ATTR << "=\"" << VOLUME_TYPE << "\" " << KEY_ATTR << "=\"" << MATRIX_KEY << "\" " << VALUE_ATTR << "=\"";
                        Transform3d matrix = volume->get_matrix() * volume->source.transform.get_matrix();
                        for (int r = 0; r < 4; ++r)
                        {
                            for (int c = 0; c < 4; ++c)
                            {
                                stream << matrix(r, c);
                                if ((r != 3) || (c != 3))
                                    stream << " ";
                            }
                        }
                        stream << "\"/>\n";
                        
                        // stores volume's source data
                        {
                            std::string input_file = xml_escape(m_fullpath_sources ? volume->source.input_file : boost::filesystem::path(volume->source.input_file).filename().string());
                            std::string prefix = std::string("   <") + METADATA_TAG + " " + TYPE_ATTR + "=\"" + VOLUME_TYPE + "\" " + KEY_ATTR + "=\"";
                            if (! volume->source.input_file.empty()) {
                                stream << prefix << SOURCE_FILE_KEY      << "\" " << VALUE_ATTR << "=\"" << input_file << "\"/>\n";
                                stream << prefix << SOURCE_OBJECT_ID_KEY << "\" " << VALUE_ATTR << "=\"" << volume->source.object_idx << "\"/>\n";
                                stream << prefix << SOURCE_VOLUME_ID_KEY << "\" " << VALUE_ATTR << "=\"" << volume->source.volume_idx << "\"/>\n";
                                stream << prefix << SOURCE_OFFSET_X_KEY  << "\" " << VALUE_ATTR << "=\"" << volume->source.mesh_offset(0) << "\"/>\n";
                                stream << prefix << SOURCE_OFFSET_Y_KEY  << "\" " << VALUE_ATTR << "=\"" << volume->source.mesh_offset(1) << "\"/>\n";
                                stream << prefix << SOURCE_OFFSET_Z_KEY  << "\" " << VALUE_ATTR << "=\"" << volume->source.mesh_offset(2) << "\"/>\n";
                                stream << prefix << SOURCE_OFFSET_Z_KEY  << "\" " << VALUE_ATTR << "=\"" << volume->source.mesh_offset(2) << "\"/>\n";
                            }
                            if (volume->source.is_converted_from_inches)
                                stream << prefix << SOURCE_IN_INCHES << "\" " << VALUE_ATTR << "=\"1\"/>\n";
                        }
                        
                        // stores volume's config data
                        for (const std::string& key : volume->config.keys())
                        {
                            stream << "   <" << METADATA_TAG << " " << TYPE_ATTR << "=\"" << VOLUME_TYPE << "\" " << KEY_ATTR << "=\"" << key << "\" " << VALUE_ATTR << "=\"" << volume->config.opt_serialize(key) << "\"/>\n";
                        }
                        
                        stream << "  </" << VOLUME_TAG << ">\n";
                    }
                }
            }
            
            stream << " </" << OBJECT_TAG << ">\n";
        }
    }
    
    stream << "</" << CONFIG_TAG << ">\n";
    
    std::string out = stream.str();
    
    if (!mz_zip_writer_add_mem(&archive, MODEL_CONFIG_FILE.c_str(), (const void*)out.data(), out.length(), MZ_DEFAULT_COMPRESSION))
    {
        add_error("Unable to add model config file to archive");
        return false;
    }
    
    return true;
}

bool _3MF_Exporter::_add_custom_gcode_per_print_z_file_to_archive( mz_zip_archive& archive, Model& model, const DynamicPrintConfig* config)
{
    std::string out = "";
    
    if (!model.custom_gcode_per_print_z.gcodes.empty())
    {
        pt::ptree tree;
        pt::ptree& main_tree = tree.add("custom_gcodes_per_print_z", "");
        
        for (const CustomGCode::Item& code : model.custom_gcode_per_print_z.gcodes)
        {
            pt::ptree& code_tree = main_tree.add("code", "");
            
            // store data of custom_gcode_per_print_z
            code_tree.put("<xmlattr>.print_z"   , code.print_z  );
            code_tree.put("<xmlattr>.type"      , static_cast<int>(code.type));
            code_tree.put("<xmlattr>.extruder"  , code.extruder );
            code_tree.put("<xmlattr>.color"     , code.color    );
            code_tree.put("<xmlattr>.extra"     , code.extra    );
            
            // add gcode field data for the old version of the PrusaSlicer
            std::string gcode = code.type == CustomGCode::ColorChange ? config->opt_string("color_change_gcode")    :
            code.type == CustomGCode::PausePrint  ? config->opt_string("pause_print_gcode")     :
            code.type == CustomGCode::Template    ? config->opt_string("template_custom_gcode") :
            code.type == CustomGCode::ToolChange  ? "tool_change"   : code.extra;
            code_tree.put("<xmlattr>.gcode"     , gcode   );
        }
        
        pt::ptree& mode_tree = main_tree.add("mode", "");
        // store mode of a custom_gcode_per_print_z
        mode_tree.put("<xmlattr>.value", model.custom_gcode_per_print_z.mode == CustomGCode::Mode::SingleExtruder ? CustomGCode::SingleExtruderMode :
                      model.custom_gcode_per_print_z.mode == CustomGCode::Mode::MultiAsSingle ?  CustomGCode::MultiAsSingleMode :
                      CustomGCode::MultiExtruderMode);
        
        if (!tree.empty())
        {
            std::ostringstream oss;
            boost::property_tree::write_xml(oss, tree);
            out = oss.str();
            
            // Post processing("beautification") of the output string
            boost::replace_all(out, "><", ">\n<");
        }
    }
    
    if (!out.empty())
    {
        if (!mz_zip_writer_add_mem(&archive, CUSTOM_GCODE_PER_PRINT_Z_FILE.c_str(), (const void*)out.data(), out.length(), MZ_DEFAULT_COMPRESSION))
        {
            add_error("Unable to add custom Gcodes per print_z file to archive");
            return false;
        }
    }
    
    return true;
}
#endif

bool load_rmf(const char* path, DynamicPrintConfig& config, ConfigSubstitutionContext& config_substitutions, Model* model, bool check_version)
{
    if (path == nullptr || model == nullptr)
        return false;
    
    _RMF_Importer importer;
    bool res = importer.load_model_from_file(path, *model, config, config_substitutions, check_version);
    importer.log_errors();
    return res;
}

bool store_rmf(const char* path, Model* model, const DynamicPrintConfig* config, bool fullpath_sources, const ThumbnailData* thumbnail_data)
{
    if ((path == nullptr) || (model == nullptr))
        return false;
    
    return false; // no exporter for now
    /* _3MF_Exporter exporter;
     bool res = exporter.save_model_to_file(path, *model, config, fullpath_sources, thumbnail_data);
     if (!res)
     exporter.log_errors();
     
     return res; */
}
} // namespace Slic3r
