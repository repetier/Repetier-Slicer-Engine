#ifndef slic3r_Format_rmf_hpp_
#define slic3r_Format_rmf_hpp_

namespace Slic3r {

/* The format for saving the SLA points was changing in the past. This enum holds the latest version that is being currently used.
 * Examples of the Slic3r_PE_sla_support_points.txt for historically used versions:
 
 *  version 0 : object_id=1|-12.055421 -2.658771 10.000000
 object_id=2|-14.051745 -3.570338 5.000000
 // no header and x,y,z positions of the points)
 
 * version 1 :  ThreeMF_support_points_version=1
 object_id=1|-12.055421 -2.658771 10.000000 0.4 0.0
 object_id=2|-14.051745 -3.570338 5.000000 0.6 1.0
 // introduced header with version number; x,y,z,head_size,is_new_island)
 */

class Model;
struct ConfigSubstitutionContext;
class DynamicPrintConfig;
struct ThumbnailData;


class RMFInStream {
public:
    uint32_t id;
    uint32_t length;
    uint32_t bits;
    uint32_t bitsLeft;
    uint32_t pos;
    std::ifstream in;
    uint32_t version;
    uint32_t bitBuffer;
    
    inline void readFloat(float &x) {in.read((char*)&x,sizeof(x));pos+=4;}
    inline void readInt32(int32_t &x) {in.read((char*)&x,sizeof(x));pos+=4;}
    inline void readUInt32(uint32_t &x) {in.read((char*)&x,sizeof(x));pos+=4;}
    inline void readInt16(int16_t &x) {in.read((char*)&x,sizeof(x));pos+=2;}
    void readString(std::string &s);
    void readBits(uint32_t &x);
    bool connect(std::string filename);
    void toEndOfBlock();
    void scanBlockType();
};

class RMFOutStream {
public:
    uint32_t id;
    uint32_t length;
    uint32_t bits;
    char *data;
    size_t used; // data used
    size_t reserved; // data reserved
    uint32_t bitsWritten;
    uint32_t bitBuffer;
    uint32_t bitMask;
    
    RMFOutStream();
    ~RMFOutStream();
    inline void writeFloat(float &x) {write((char*)&x,sizeof(x));}
    inline void writeInt32(int32_t &x) {write((char*)&x,sizeof(x));}
    inline void writeUInt32(uint32_t &x) {write((char*)&x,sizeof(x));}
    inline void writeInt16(int16_t &x) {write((char*)&x,sizeof(x));}
    inline void writeString(std::string s) {write(s.c_str(), s.length() + 1);}
    void computeBitsForMax(uint32_t m);
    void startBlock(uint32_t _id);
    void writeBits(uint32_t val);
    void writeBlock(std::ofstream &out);
    void write(const char *ptr, size_t sz);
};

// Load the content of a 3mf file into the given model and preset bundle.
extern bool load_rmf(const char* path, DynamicPrintConfig& config, ConfigSubstitutionContext& config_substitutions, Model* model, bool check_version);

// Save the given model and the config data contained in the given Print into a 3mf file.
// The model could be modified during the export process if meshes are not repaired or have no shared vertices
extern bool store_rmf(const char* path, Model* model, const DynamicPrintConfig* config, bool fullpath_sources, const ThumbnailData* thumbnail_data = nullptr);

} // namespace Slic3r

#endif /* slic3r_Format_3mf_hpp_ */
