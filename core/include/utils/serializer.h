#pragma once
#include <algorithm>
#include <map>
#include <string>
#include <vector>

const char serialization_separator = '$';
const std::size_t MAX_FILE_SIZE = 2048;

class Serializer {
private:
    std::map<std::string, int> ints;
    std::map<std::string, bool> bools;
    std::map<std::string, double> doubles;
    std::map<std::string, std::string> strings;

public:

    
    /// @brief create an empty Serializer
    Serializer() : ints({}), bools({}), doubles({}), strings({}) {}

    /// @brief create a Serializer filled with values from file
    explicit Serializer(const std::string &filename) { read_from_disk(filename); }

    /// @brief saves current Serializer state to disk
    void save_to_disk(const std::string &filename) const;
    /// @brief loads Serializer state from disk
    bool read_from_disk(const std::string &filename);

    /// Setters - not saved until save_to_disk is called

    void set_int(const std::string &name, int i);
    void set_bool(const std::string &name, bool b);
    void set_double(const std::string &name, double d);
    void set_string(const std::string &name, std::string str);

    /// Getters
    /// Return value if it exists in the serializer

int int_or(const std::string &name, int otherwise) const;
bool bool_or(const std::string &name, bool otherwise) const;
double double_or(const std::string &name, double otherwise) const;
std::string string_or(const std::string &name, std::string otherwise) const;
};
