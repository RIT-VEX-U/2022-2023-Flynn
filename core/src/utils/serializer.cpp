#include "../core/include/utils/serializer.h"
#include "stdlib.h"
#include "vex.h"

// ===================== Helper Functions for converting to and from bytes =====================
// Specialize these if you have non trivial types you need to save I suppose

// Convert a type to bytes to serialize

template<typename T>
std::vector<char> to_bytes(T value) {
    // copy bytes of data into vector
    std::vector<char> value_bytes(sizeof(T));
    std::copy(static_cast<const char *>(static_cast<const void *>(&value)),
              static_cast<const char *>(static_cast<const void *>(&value)) + sizeof(value),
              value_bytes.begin());
    return value_bytes;
}

template<>
std::vector<char> to_bytes<std::string>(std::string str) {
    std::vector<char> value_bytes(str.size() + 1);
    std::copy(str.begin(), str.end(), value_bytes.begin());
    value_bytes[str.size()] = 0x0;
    return value_bytes;
}

// Convert bytes to a type

template<typename T>
T from_bytes(const std::vector<char> &data, std::vector<char>::const_iterator &position) {
    T value;
    std::copy(position, position + sizeof(T), static_cast<char *>(static_cast<void *>(&value)));
    position = position + sizeof(T);
    return value;
}

template<>
std::string from_bytes(const std::vector<char> &data, std::vector<char>::const_iterator &position) {
    auto pos = position;
    while (*pos != 0x0) {
        ++pos;
    }
    std::string s(position, pos);
    position = pos + 1;
    return s;
}

// Replaces funny characters in names so they don't mess with serialization specifiers

std::string sanitize_name(std::string s) {
    std::replace(s.begin(), s.end(), serialization_separator, '-');
    return s;
}


// Protocol
/*
     * Null terminated name immediatly followed by the bytes of the value
     * +----Ints-----+
     * |name\0 bytes |
     * |   [sep]     |
     * +---Bools-----+
     * |name\0 byte  |
     * |   [sep]     |
     * +---Doubles --+
     * |name\0 bytes |
     * |   [sep]     |
     * +---Strings --+
     * |name\0 str\0 |
     * |   [sep]     |
     * +-------------+
    */

// Adds data to a file (represented by array of bytes) as specified by the format above
template<typename value_type>
static void add_data(std::vector<char> &data, const std::map<std::string, value_type> &map) {
    for (const auto &pair : map) {
        const std::string &name = pair.first;
        const value_type value = pair.second;

        data.insert(data.end(), name.cbegin(), name.cend());
        data.insert(data.end(), 0x0);

        auto bytes = to_bytes<value_type>(value);
        data.insert(data.end(), bytes.cbegin(), bytes.cend());
    }
    data.push_back(serialization_separator);
}

// reads data of a certain type from a file
template<typename value_type>
static std::vector<char>::const_iterator read_data(const std::vector<char> &data,
                                                   std::vector<char>::const_iterator begin,
                                                   std::map<std::string, value_type> &map) {
    std::vector<char>::const_iterator pos = begin;

    while (*pos != serialization_separator) {
        auto name_start = pos;
        // read name
        std::string name = from_bytes<std::string>(data, name_start);
        pos += name.size() + 1;
        // read value
        value_type value = from_bytes<value_type>(data, pos);
        map.insert({name, value});
    }

    return pos + 1;
}

void Serializer::set_int(const std::string &name, int i) {
    ints[sanitize_name(name)] = i;
}
void Serializer::set_bool(const std::string &name, bool b) {
    bools[sanitize_name(name)] = b;
}
void Serializer::set_double(const std::string &name, double d) {
    doubles[sanitize_name(name)] = d;
}
void Serializer::set_string(const std::string &name, std::string str) {
    strings[sanitize_name(name)] = str;
}

int Serializer::int_or(const std::string &name, int otherwise) const {
    if (ints.count(name)) {
        return ints.at(name);
    }
    return otherwise;
}
bool Serializer::bool_or(const std::string &name, bool otherwise) const {
    if (bools.count(name)) {
        return bools.at(name);
    }
    return otherwise;
}
double Serializer::double_or(const std::string &name, double otherwise) const {
    if (doubles.count(name)) {
        return doubles.at(name);
    }
    return otherwise;
}
std::string Serializer::string_or(const std::string &name, std::string otherwise) const {
    if (strings.count(name)) {
        return strings.at(name);
    }
    return otherwise;
}

// forms data bytes then saves to a filename
void Serializer::save_to_disk(const std::string &filename) const {
    std::vector<char> data = {};
    add_data<int>(data, ints);
    add_data<bool>(data, bools);
    add_data<double>(data, doubles);
    add_data<std::string>(data, strings);

    vex::brain::sdcard sd;
    sd.savefile(filename.c_str(), (unsigned char *)&data[0], data.size());
}

// reads types from file data
bool Serializer::read_from_disk(const std::string &filename) {
    vex::brain::sdcard sd;
    int size = sd.size(filename.c_str());
    std::vector<char> data(size);
    // std::vector<char> data = loadFile(filename);

    int readsize = sd.loadfile(filename.c_str(), (unsigned char *)&data[0], size);
    if (size!=readsize){
        return false;
    }




    auto bool_start = read_data<int>(data, data.cbegin(), ints);
    auto doubles_start = read_data<bool>(data, bool_start, bools);
    auto strings_start = read_data<double>(data, doubles_start, doubles);
    read_data<std::string>(data, strings_start, strings);

    return true;
}
